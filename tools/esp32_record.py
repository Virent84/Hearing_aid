#!/usr/bin/env python3
"""Record the 4 ESP32 microphones to a 4-channel 16-bit WAV (sync offset from config.json already applied).
   python3 tools/esp32_record.py --seconds 10 --out data/mics4ch.wav [--port /dev/ttyACM0]
Then:  python3 tools/offline_wav.py data/mics4ch.wav --expect 0"""
import argparse, os, sys, time, wave
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from ha import config
from ha.audio_source import ESP32Source

ap = argparse.ArgumentParser(); ap.add_argument("--port"); ap.add_argument("--seconds", type=float, default=10)
ap.add_argument("--out", default="data/mics4ch.wav"); ap.add_argument("--config"); a = ap.parse_args()
cfg = config.load(a.config); fs = cfg["sample_rate"]; src = ESP32Source(cfg, a.port)
t0 = time.time()
while not src.stat["link"] and time.time() - t0 < 10: time.sleep(0.1)
if not src.stat["link"]: sys.exit("No data from the ESP32 - run tools/esp32_check.py for diagnostics")
print(f"Recording {a.seconds:.0f} s ... speak now"); n = int(a.seconds * fs); got = 0; ch = []
while got < n:
    x = src.read(1024); ch.append(x); got += x.shape[1]
x = np.concatenate(ch, axis=1)[:, :n]; st = src.stats(); src.close()
os.makedirs(os.path.dirname(a.out) or ".", exist_ok=True)
with wave.open(a.out, "wb") as w:
    w.setnchannels(4); w.setsampwidth(2); w.setframerate(fs)
    w.writeframes((np.clip(x, -1, 1) * 32767).round().astype("<i2").T.copy().tobytes())
print(f"wrote {a.out}   lost packets: {st['lost']}  crc errors: {st['crc_err']}")
print("levels dBFS: " + "  ".join(f"M{i+1}:{20*np.log10(np.sqrt(np.mean(x[i]**2))+1e-12):.1f}" for i in range(4)))
