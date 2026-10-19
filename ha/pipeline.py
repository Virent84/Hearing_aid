"""Real-time pipeline: source -> preprocess -> STFT -> NR -> GCC-PHAT/DOA -> beamformer -> output."""
import threading, time, subprocess, collections, wave
import numpy as np
from .geometry import Array
from .dsp import Preprocessor, NoiseReducer, GccDoa, Beamformer, STFT


def angdiff(a, b):
    return (a - b + 180) % 360 - 180


class Pipeline:
    def __init__(self, cfg, source):
        self.cfg = cfg; self.src = source
        self.fs = cfg["sample_rate"]; N = cfg["stft"]["frame"]; H = cfg["stft"]["hop"]
        self.arr = Array(cfg); M = self.arr.M; F = N // 2 + 1; self.H = H
        self.pre = Preprocessor(cfg, M); self.stft = STFT(N, H, M)
        self.nr = NoiseReducer(cfg, F); self.doa = GccDoa(cfg, self.arr, F, self.fs, N)
        self.bf = Beamformer(cfg, self.arr, F, self.fs, N)
        self.method = cfg["beamformer"]["method"]
        self.noise_db = -80.0; self.vad = False
        self.speaker = 0.0; self.speaker_conf = 0.0; self.beam = 0.0; self.locked = None
        self.out_gain = 10 ** (cfg["output"]["gain_db"] / 20); self.agc = 1.0
        self.state = {"status": "starting"}; self.log = collections.deque(maxlen=60)
        self.lock = threading.Lock(); self.running = False; self.frames = 0
        self.playback = cfg["output"]["playback"]; self._ap = None; self.wav = None
        self.out_chunks = collections.deque(maxlen=64); self.proc_ms = 0.0
        self.tdoa = [0.0] * len(self.arr.pairs); self.mic_db = [-90.0] * M; self.out_db = -90.0
        self.on_frame = None   # optional hook(in_hop[M,H], out_hop[H])
        self.link = None; self._link_prev = None
        self.say("pipeline ready: " + getattr(source, "name", "?"))

    def say(self, msg):
        self.log.appendleft(f"{time.strftime('%H:%M:%S')}  {msg}")

    # --- controls -------------------------------------------------------------------
    def control(self, d):
        if "method" in d and d["method"] in ("mvdr", "das"):
            self.method = self.bf.method = d["method"]; self.say(f"beamformer -> {d['method'].upper()}")
        if "lock" in d:
            self.locked = None if d["lock"] is None else float(d["lock"]) % 360
            self.say("beam locked to %.0f°" % self.locked if self.locked is not None else "beam follows speaker")
        if "playback" in d:
            self.set_playback(bool(d["playback"]))
        if "noise_reduction" in d:
            self.nr.on = bool(d["noise_reduction"]); self.say(f"noise reduction {'on' if self.nr.on else 'off'}")

    def set_playback(self, on):
        self.playback = on
        if on and self._ap is None:
            o = self.cfg["output"]
            try:
                self._ap = subprocess.Popen(["aplay", "-q", "-D", o["device"], "-t", "raw", "-f", "S16_LE", "-c", "1", "-r", str(self.fs)],
                                            stdin=subprocess.PIPE)
            except Exception as e:
                self.say(f"playback unavailable: {e}"); self.playback = False
        if not on and self._ap:
            self._ap.kill(); self._ap = None
        self.say(f"playback {'on' if self.playback else 'off'}")

    # --- processing -----------------------------------------------------------------
    def step(self, x):
        t0 = time.perf_counter(); cfg = self.cfg
        x = self.pre(x, active=not self.vad)
        X = self.stft.analyze(x)
        self.mic_db = (10 * np.log10(np.mean(x ** 2, axis=1) + 1e-12)).tolist()
        Xd = self.nr(X)
        # VAD: band energy of raw frame vs min-statistics noise floor
        e = np.mean(np.abs(X[:, self.doa.band]) ** 2); n = np.mean(self.nr.noise[self.doa.band]) / 1.5
        snr_db = 10 * np.log10(e / (n + 1e-12) + 1e-12)
        self.vad = snr_db > cfg["doa"]["vad_margin_db"]; self.snr_db = snr_db
        if self.vad:
            self.doa.update(Xd)
        ang, conf = self.doa.scan() if (self.frames % 4 == 0) else (self.speaker, self.speaker_conf)
        if self.vad and conf >= cfg["doa"]["min_confidence"]:
            # circular smoothing
            self.speaker = (self.speaker + 0.35 * angdiff(ang, self.speaker)) % 360
        self.speaker_conf = conf
        # beam steering w/ hysteresis + slew limit
        target = self.locked if self.locked is not None else self.speaker
        diff = angdiff(target, self.beam)
        if abs(diff) > cfg["beamformer"]["hysteresis_deg"] or self.locked is not None:
            mx = cfg["beamformer"]["max_slew_deg_per_s"] * self.H / self.fs
            self.beam = (self.beam + np.clip(diff, -mx, mx)) % 360
        Y = self.bf(Xd, self.beam, adapt=True)
        y = self.stft.synth(Y)
        # output AGC (slow) + gain
        lvl = np.sqrt(np.mean(y ** 2)) + 1e-9
        self.agc = 0.98 * self.agc + 0.02 * float(np.clip(0.08 / lvl, 0.5, 8.0))
        out = np.clip(y * self.agc * self.out_gain, -1, 1).astype(np.float32)
        self.out_db = float(20 * np.log10(np.sqrt(np.mean(out ** 2)) + 1e-9))
        self.frames += 1
        if self.frames % 20 == 0:
            self.tdoa = self.doa.pair_tdoa()
        self.proc_ms = 0.9 * self.proc_ms + 0.1 * (time.perf_counter() - t0) * 1000
        return out

    def _emit(self):
        with self.lock:
            self.state = {
                "status": "speech" if self.vad else "listening", "source": getattr(self.src, "name", "?"),
                "speaker": round(self.speaker, 1), "beam": round(float(self.beam), 1), "conf": round(self.speaker_conf, 3),
                "locked": self.locked, "method": self.method, "nr": self.nr.on, "playback": self.playback,
                "snr_db": round(float(getattr(self, "snr_db", 0)), 1), "mic_db": [round(v, 1) for v in self.mic_db],
                "out_db": round(self.out_db, 1), "proc_ms": round(self.proc_ms, 2), "budget_ms": round(1000 * self.H / self.fs, 1),
                "frames": self.frames, "tdoa_us": [round(t * 1e6, 1) for t in self.tdoa],
                "pairs": [[i + 1, j + 1] for i, j in self.arr.pairs],
                "srp": np.round(self.doa.score, 3).tolist(), "srp_step": self.cfg["doa"]["angle_step_deg"],
                "mics": [{"id": i + 1, "angle": float(np.rad2deg(a))} for i, a in enumerate(self.arr.angles)],
                "radius_mm": self.cfg["array"]["radius_m"] * 1000,
                "truth": getattr(self.src, "truth", None), "link": self.link, "log": list(self.log),
            }

    def run(self):
        self.running = True; self.say("running"); last = 0; lastbeam = None; lastvad = None
        wav = None
        if self.cfg["output"]["record_wav"]:
            wav = wave.open(self.cfg["output"]["record_wav"], "wb"); wav.setnchannels(1); wav.setsampwidth(2); wav.setframerate(self.fs)
        if self.playback: self.set_playback(True)
        try:
            while self.running:
                x = self.src.read(self.H)
                out = self.step(x)
                if self.on_frame: self.on_frame(x, out)
                pcm = (out * 32767).astype(np.int16).tobytes()
                if wav: wav.writeframes(pcm)
                if self._ap and self.playback:
                    try: self._ap.stdin.write(pcm)
                    except Exception: self.set_playback(False)
                if hasattr(self.src, "stats") and self.frames % 25 == 0:
                    self.link = self.src.stats(); lk = self.link
                    key = (lk["link"], lk["lost"] > 0, lk["crc_err"] > 0)
                    if key != self._link_prev:
                        self.say(("ESP32 link UP" if lk["link"] else "ESP32 link DOWN (no data)") +
                                 f"  lost={lk['lost']} crc_err={lk['crc_err']}")
                        self._link_prev = key
                if self.vad != lastvad:
                    self.say("speech detected" if self.vad else "silence / noise only"); lastvad = self.vad
                if lastbeam is None or abs(angdiff(self.beam, lastbeam)) > 15:
                    self.say(f"beam -> {self.beam:.0f}° (speaker {self.speaker:.0f}°)"); lastbeam = self.beam
                if time.time() - last > 0.06:
                    self._emit(); last = time.time()
        except Exception as e:
            self.say(f"ERROR: {e}"); self.state["status"] = "error"; self._emit(); raise
        finally:
            if wav: wav.close()
            self.src.close()

    def start(self):
        th = threading.Thread(target=self.run, daemon=True); th.start(); return th
