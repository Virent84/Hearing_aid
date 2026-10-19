#!/usr/bin/env python3
"""ESP32 link + microphone sync check.  Run with the ESP32 plugged into the Pi, then CLAP sharply
3-4 times directly above the centre of the array during the test.
   python3 tools/esp32_check.py [--port /dev/ttyACM0] [--seconds 8]
Reports: packet loss, CRC errors, true sample rate, per-mic level/DC/clipping, inter-mic lags at claps,
and the config.json value to use for esp32.port1_shift_samples."""
import argparse, os, sys, time
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from ha import config
from ha.audio_source import ESP32Source

ap = argparse.ArgumentParser(); ap.add_argument("--port"); ap.add_argument("--seconds", type=float, default=8)
ap.add_argument("--config"); a = ap.parse_args()
cfg = config.load(a.config); fs = cfg["sample_rate"]
src = ESP32Source(cfg, a.port)
shift_now = src.shift
print(f"Waiting for ESP32 data ({src.port_cfg}) ...")
t0 = time.time()
while not src.stat["link"] and time.time() - t0 < 10: time.sleep(0.1)
if not src.stat["link"]:
    sys.exit("NO DATA from the ESP32.\n  - cable in the native 'USB' port? data cable (not charge-only)?\n"
             "  - ls /dev/ttyACM* ; lsusb | grep 303a ; groups | grep dialout\n  - firmware flashed? (USB CDC On Boot = Enabled)")
print(f"Recording {a.seconds:.0f} s from {src.stat['port']} - clap now ...")
chunks = []; t1 = time.time(); n_total = int(a.seconds * fs); got = 0
while got < n_total:
    x = src.read(1024); chunks.append(x); got += x.shape[1]
elapsed = time.time() - t1
st = src.stats(); x = np.concatenate(chunks, axis=1); src.close()

ok = True
def flag(cond, msg):
    global ok
    if not cond: ok = False
    print(("  OK    " if cond else "  FAIL  ") + msg)

print("\\n== Link ==".replace("\\n", "\n"))
print(f"  packets {st['packets']}  lost {st['lost']}  crc_err {st['crc_err']}  resync {st['resync']}  "
      f"esp_overflow {st['esp_overflow']}  esp_usb_drop {st['esp_usb_drop']}  pi_overrun {st['pi_overrun']}")
rate = st["samples"] / max(time.time() - t0 - 0.0, 1e-9)
flag(st["lost"] == 0 and st["crc_err"] == 0, "no lost packets / CRC errors")
flag(st["esp_overflow"] == 0 and st["esp_usb_drop"] == 0, "no I2S overflow / USB drops on the ESP32")

print("\n== Microphones (rms dBFS, DC, peak) ==")
for i in range(4):
    r = np.sqrt(np.mean(x[i] ** 2)); db = 20 * np.log10(r + 1e-12)
    print(f"  M{i+1}: {db:6.1f} dBFS   DC {np.mean(x[i]):+.4f}   peak {np.abs(x[i]).max():.3f}")
    flag(db > -80, f"M{i+1} is alive (not silent)")
    flag(np.abs(x[i]).max() < 0.98, f"M{i+1} not clipping")
lv = [20 * np.log10(np.sqrt(np.mean(x[i] ** 2)) + 1e-12) for i in range(4)]
flag(max(lv) - min(lv) < 8, f"mic levels within 8 dB of each other (spread {max(lv)-min(lv):.1f} dB)")

print("\n== Sync at claps (lag in samples; + means the second mic hears it LATER) ==")
d = np.abs(np.diff(x, axis=1)); env = d.max(axis=0)
ev = []
if env.max() > 10 * (np.median(env) + 1e-9):
    for idx in np.argsort(-env):
        if env[idx] < 0.3 * env.max() or len(ev) >= 6: break
        if all(abs(idx - e) > 0.15 * fs for e in ev) and 80 < idx < x.shape[1] - 80: ev.append(int(idx))
if not ev:
    print("  No clear claps detected. Clap sharply (3-4 times) above the array centre and run again.")
else:
    pairs = [(0, 1), (2, 3), (0, 2), (0, 3), (1, 2), (1, 3)]
    L = {p: [] for p in pairs}
    for p0 in ev:
        w = np.diff(x[:, p0 - 48:p0 + 49], axis=1)
        for i, j in pairs:
            c = np.correlate(w[j], w[i], "full"); mid = len(w[i]) - 1
            lag = np.argmax(c[mid - 12:mid + 13]) - 12
            L[(i, j)].append(int(lag))
    print(f"  {len(ev)} claps used")
    for (i, j), v in L.items():
        m = float(np.median(v)); tag = "same I2S port" if (i, j) in [(0, 1), (2, 3)] else "ACROSS I2S ports"
        print(f"  M{i+1}->M{j+1}: lags {v}  median {m:+.0f}   ({tag})")
        flag(abs(m) <= 1, f"M{i+1}/M{j+1} aligned within +-1 sample")
    cross = [np.median(L[p]) for p in [(0, 2), (0, 3), (1, 2), (1, 3)]]
    off = int(round(float(np.mean(cross))))
    print(f"\n  Port B (M3/M4) vs port A (M1/M2) residual offset: {np.mean(cross):+.2f} samples")
    if off != 0:
        print(f"  >>> set  \"port1_shift_samples\": {shift_now + off}  in config.json (esp32 section; currently {shift_now}) and re-run this check.")
    else:
        print(f"  Ports are aligned with port1_shift_samples = {shift_now}.")
print("\nRESULT: " + ("PASS" if ok else "CHECK THE FAIL LINES ABOVE"))
sys.exit(0 if ok else 1)
