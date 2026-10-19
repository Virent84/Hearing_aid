"""Audio sources returning float32 arrays [M, n] in [-1,1]."""
import subprocess, time, wave, re, threading, struct, binascii, glob
import numpy as np
from .geometry import Array

class ALSASource:
    def __init__(self, cfg):
        a = cfg["alsa"]; self.fs = cfg["sample_rate"]; self.nch = a["channels"]
        self.M = len(cfg["array"]["mic_angles_deg"]); self.bits = a["bits"]
        self.dev = a["device"] if a["device"] != "auto" else self._find_device()
        self.fmt = a["format"]
        self.map = a["channel_map"]
        self.buf = b""
        self.proc = subprocess.Popen(
            ["arecord", "-q", "-D", self.dev, "-c", str(self.nch), "-r", str(self.fs),
             "-f", self.fmt, "-t", "raw", "--buffer-size", "16384"],
            stdout=subprocess.PIPE, bufsize=0)
        if self.map == "auto":
            self.map = self._probe()
        self.map = list(self.map)
        self.name = f"ALSA {self.dev} ch{self.map}"

    @staticmethod
    def _find_device():
        out = subprocess.run(["arecord", "-l"], capture_output=True, text=True).stdout
        if "mic4" in out:
            return "hw:CARD=mic4,DEV=0"
        for line in out.splitlines():
            m = re.match(r"card (\d+): (\S+) .*device (\d+)", line)
            if m and not any(k in line.lower() for k in ("hdmi", "usb", "headphones")):
                return f"hw:{m.group(1)},{m.group(3)}"
        raise RuntimeError("No I2S capture card found (arecord -l). Run tools/hw_check.py")

    def _raw(self, n):
        need = n * self.nch * 4
        while len(self.buf) < need:
            d = self.proc.stdout.read(need - len(self.buf))
            if not d:
                raise RuntimeError("arecord stopped (device busy/unsupported format?)")
            self.buf += d
        d, self.buf = self.buf[:need], self.buf[need:]
        x = np.frombuffer(d, dtype="<i4").reshape(-1, self.nch).T.astype(np.float32)
        return x / 2147483648.0

    def _probe(self):
        x = self._raw(self.fs)[:, 2000:]
        s = x.std(axis=1)
        live = [int(c) for c in np.argsort(-s)[: self.M] if s[c] > 1e-7]
        if len(live) < self.M:
            raise RuntimeError(f"Only {len(live)} live channels found (std={s}). Check wiring/overlay; run tools/hw_check.py")
        return sorted(live)

    def read(self, n):
        return self._raw(n)[self.map]

    def close(self):
        self.proc.kill()


class _Paced:
    def pace(self, n):
        self._t = getattr(self, "_t", time.time()) + n / self.fs
        d = self._t - time.time()
        if d > 0: time.sleep(d)
        elif d < -1: self._t = time.time()


class WavSource(_Paced):
    def __init__(self, cfg, path):
        w = wave.open(path); self.fs = w.getframerate(); ch = w.getnchannels()
        raw = np.frombuffer(w.readframes(w.getnframes()), "<i2" if w.getsampwidth() == 2 else "<i4")
        self.x = (raw.reshape(-1, ch).T.astype(np.float32) / (2 ** (8 * w.getsampwidth() - 1)))[: len(cfg["array"]["mic_angles_deg"])]
        self.i = 0; self.name = f"WAV {path}"

    def read(self, n):
        idx = (np.arange(self.i, self.i + n)) % self.x.shape[1]; self.i += n
        self.pace(n); return self.x[:, idx]
    def close(self): pass


class SimSource(_Paced):
    """Synthetic speech-like target + interferer + diffuse noise, exact fractional delays."""
    def __init__(self, cfg, realtime=True, seed=0):
        self.fs = cfg["sample_rate"]; self.arr = Array(cfg); self.cfg = cfg["sim"]
        self.rng = np.random.default_rng(seed); self.t = 0; self.realtime = realtime
        self.name = "SIMULATION"; self.truth = self.cfg["target_deg"]
        self.last_target = None; self.last_interf = None

    def _voice(self, n, f0, off):
        t = (np.arange(n) + off) / self.fs
        env = np.clip(np.sin(2 * np.pi * 3.1 * t + f0) + 0.4, 0, 1)
        ph = 2 * np.pi * f0 * t + 3 * np.sin(2 * np.pi * 0.7 * t)
        s = sum(np.sin(k * ph) / k ** 0.8 for k in range(1, 40) if k * f0 < self.fs / 2.2)
        return env * s

    def _delay(self, s, taus):
        n = len(s); S = np.fft.rfft(s); f = np.fft.rfftfreq(n, 1 / self.fs)
        return np.fft.irfft(S[None] * np.exp(-2j * np.pi * f[None] * taus[:, None]), n, axis=1)

    def read(self, n):
        pad = 256; N = n + 2 * pad; sc = self.cfg
        tgt = sc["target_deg"] + (35 * np.sin(2 * np.pi * self.t / self.fs / 12) if sc.get("move") else 0)
        self.truth = float(tgt % 360)
        s1 = self._voice(N, 120.0, self.t - pad)
        s2 = self._voice(N, 210.0, self.t - pad + 5000) * 0.7
        t1 = self.arr.arrival_times(np.deg2rad(tgt)); t2 = self.arr.arrival_times(np.deg2rad(sc["interferer_deg"]))
        a = self._delay(s1, t1)[:, pad:pad + n]; b = self._delay(s2, t2)[:, pad:pad + n]
        a *= 0.05 / (np.sqrt(np.mean(a ** 2)) + 1e-9)
        b *= 0.5 * 0.05 / (np.sqrt(np.mean(b ** 2)) + 1e-9)   # interferer 6 dB below target
        self.last_target, self.last_interf = a.copy(), b.copy()   # for evaluation
        noise = self.rng.standard_normal(a.shape) * 0.05 * 10 ** (-sc["snr_db"] / 20)
        self.t += n
        if self.realtime: self.pace(n)
        return (a + b + noise).astype(np.float32)
    def close(self): pass


class ESP32Source:
    """4-channel stream from the ESP32-S3 over native USB serial (see firmware/ and docs/PROTOCOL in README).

    Packet (little-endian): u16 0xA55A | u16 seq | u16 nframes | u16 flags | i16 pcm[nframes][4] | u16 crc16
    A background thread re-syncs on garbage, checks CRC, fills gaps from lost packets with silence,
    reconnects if the cable is unplugged, and feeds a short ring buffer that read(n) drains.
    """
    MAGIC = b"\x5a\xa5"; HDR = struct.Struct("<HHHH"); CH = 4

    def __init__(self, cfg, port=None):
        e = cfg.get("esp32", {})
        self.fs = cfg["sample_rate"]; self.M = len(cfg["array"]["mic_angles_deg"])
        if self.M != self.CH:
            raise RuntimeError("ESP32 firmware streams 4 channels; array.mic_angles_deg must list 4 mics")
        self.port_cfg = port or e.get("port", "auto"); self.baud = int(e.get("baud", 115200))
        self.skip = int(e.get("skip_s", 0.5) * self.fs)
        self.shift = int(e.get("port1_shift_samples", 0))
        self.gain = 10 ** (e.get("gain_db", 0) / 20)
        self.cv = threading.Condition(); self.buf = np.zeros((self.CH, 0), np.float32)
        self.max_buf = int(1.0 * self.fs); self.last_seq = None; self.t_last = 0.0
        self.stat = dict(link=False, port="", packets=0, lost=0, crc_err=0, resync=0,
                         esp_overflow=0, esp_usb_drop=0, pi_overrun=0, samples=0)
        self.name = "ESP32 (waiting for device)"; self.running = True; self._skip_left = self.skip
        self.th = threading.Thread(target=self._reader, daemon=True); self.th.start()

    # ---- device discovery -----------------------------------------------------------
    @staticmethod
    def find_port():
        try:
            from serial.tools import list_ports
            for p in list_ports.comports():
                if p.vid == 0x303A: return p.device           # Espressif
        except Exception:
            pass
        c = sorted(glob.glob("/dev/ttyACM*"))
        return c[0] if c else None

    # ---- background reader ----------------------------------------------------------
    def _set_link(self, up):
        if self.stat["link"] != up:
            self.stat["link"] = up
        if not up: self.name = "ESP32 (no data)"

    def _reader(self):
        import serial
        rx = bytearray()
        while self.running:
            ser = None
            try:
                port = self.port_cfg if self.port_cfg != "auto" else self.find_port()
                if not port:
                    self._set_link(False); time.sleep(1.0); continue
                ser = serial.Serial(port, self.baud, timeout=0.05)
                self.stat["port"] = port; self.name = f"ESP32 {port}"
                rx.clear(); self.last_seq = None; self._skip_left = self.skip
                while self.running:
                    d = ser.read(max(1, ser.in_waiting))
                    if not d:
                        if time.time() - self.t_last > 0.5: self._set_link(False)
                        continue
                    rx += d; self._parse(rx)
            except (serial.SerialException, OSError):
                self._set_link(False); time.sleep(1.0)
            finally:
                try:
                    if ser: ser.close()
                except Exception: pass

    def _parse(self, rx):
        while True:
            i = rx.find(self.MAGIC)
            if i < 0:
                if len(rx) > 1: del rx[:-1]
                return
            if i > 0:
                del rx[:i]; self.stat["resync"] += 1
            if len(rx) < 8: return
            _, seq, n, flags = self.HDR.unpack_from(rx, 0)
            if n == 0 or n > 1024:
                del rx[:2]; self.stat["resync"] += 1; continue
            total = 8 + n * self.CH * 2 + 2
            if len(rx) < total: return
            crc = struct.unpack_from("<H", rx, total - 2)[0]
            if binascii.crc_hqx(bytes(rx[2:total - 2]), 0xFFFF) != crc:
                self.stat["crc_err"] += 1; del rx[:2]; continue
            pcm = np.frombuffer(bytes(rx[8:total - 2]), "<i2").reshape(n, self.CH).T.astype(np.float32) / 32768.0
            del rx[:total]
            self._push(seq, flags, pcm * self.gain)

    def _push(self, seq, flags, pcm):
        st = self.stat; n = pcm.shape[1]
        if self.last_seq is not None:
            gap = (seq - self.last_seq - 1) & 0xFFFF
            if 0 < gap < 100:
                st["lost"] += gap
                pcm = np.concatenate([np.zeros((self.CH, gap * n), np.float32), pcm], axis=1)   # keep timing
            elif gap >= 100:
                pass                                                                           # device restarted
        self.last_seq = seq
        st["packets"] += 1; st["samples"] += n
        if flags & 1: st["esp_overflow"] += 1
        if flags & 2: st["esp_usb_drop"] += 1
        self.t_last = time.time(); self.stat["link"] = True
        if self._skip_left > 0:                                  # ignore microphone start-up transient
            k = min(self._skip_left, pcm.shape[1]); self._skip_left -= k; pcm = pcm[:, k:]
            if pcm.shape[1] == 0: return
        with self.cv:
            self.buf = np.concatenate([self.buf, pcm], axis=1)
            if self.buf.shape[1] > self.max_buf:
                st["pi_overrun"] += self.buf.shape[1] - self.max_buf; self.buf = self.buf[:, -self.max_buf:]
            self.cv.notify_all()

    # ---- consumer API ---------------------------------------------------------------
    def read(self, n):
        k = self.shift; need = n + abs(k); t0 = time.time()
        with self.cv:
            while self.buf.shape[1] < need:
                self.cv.wait(timeout=0.25)
                if time.time() - t0 > 1.0 and self.buf.shape[1] < need:
                    return np.zeros((self.CH, n), np.float32)          # link down: feed silence, keep dashboard alive
            b = self.buf
            if k == 0:   out = b[:, :n]
            elif k > 0:  out = np.concatenate([b[0:2, :n], b[2:4, k:k + n]], axis=0)     # M3/M4 (GPIO7) lag M1/M2
            else:        out = np.concatenate([b[0:2, -k:-k + n], b[2:4, :n]], axis=0)   # M1/M2 (GPIO6) lag M3/M4
            out = out.copy(); self.buf = b[:, n:]
        return out

    def stats(self):
        s = dict(self.stat); s["buffered_ms"] = round(1000 * self.buf.shape[1] / self.fs, 1); return s

    def close(self):
        self.running = False
