#!/usr/bin/env python3
"""Run the full DOA + beamforming pipeline on a 4-channel WAV (e.g. recorded from the ESP32),
as fast as the CPU allows (no real-time pacing). Prints DOA statistics and writes the enhanced audio.
   python3 tools/offline_wav.py data/mics4ch.wav [--method mvdr|das] [--out data/output/enhanced_audio.wav] [--expect 0]
"""
import argparse, os, sys, wave, copy
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from ha import config
from ha.pipeline import Pipeline, angdiff

ap = argparse.ArgumentParser()
ap.add_argument("wav"); ap.add_argument("--config"); ap.add_argument("--method", choices=["mvdr", "das"])
ap.add_argument("--out", default="data/output/enhanced_audio.wav")
ap.add_argument("--expect", type=float, help="true speaker angle in degrees (0 = MIC1 side, CCW) to score the DOA")
ap.add_argument("--no-nr", action="store_true")
a = ap.parse_args()

cfg = config.load(a.config)
if a.method: cfg["beamformer"]["method"] = a.method
if a.no_nr: cfg["preprocess"]["noise_reduction"] = False

w = wave.open(a.wav); fs = w.getframerate(); ch = w.getnchannels(); sw = w.getsampwidth()
raw = np.frombuffer(w.readframes(w.getnframes()), "<i2" if sw == 2 else "<i4")
x = raw.reshape(-1, ch).T.astype(np.float32) / (2 ** (8 * sw - 1))
M = len(cfg["array"]["mic_angles_deg"])
if ch < M: sys.exit(f"WAV has {ch} channels, need {M}")
x = x[:M]
if fs != cfg["sample_rate"]:
    sys.exit(f"WAV is {fs} Hz but config sample_rate is {cfg['sample_rate']}. Set sample_rate in config.json to {fs} "
             f"(or resample) - a wrong rate gives wrong DOA angles.")
print(f"{a.wav}: {x.shape[1]/fs:.1f} s, {M} ch @ {fs} Hz")
print("mic levels (dBFS rms): " + "  ".join(f"M{i+1}:{20*np.log10(np.sqrt(np.mean(x[i]**2))+1e-12):6.1f}" for i in range(M)))
if np.abs(x).max() > 0.98: print("WARNING: clipping in input")

class Null:
    name = "OFFLINE"
    def close(self): pass
p = Pipeline(cfg, Null()); H = cfg["stft"]["hop"]
n = x.shape[1] // H
out = np.zeros(n * H, np.float32); ang = []; conf = []; vadf = []
for k in range(n):
    out[k*H:(k+1)*H] = p.step(x[:, k*H:(k+1)*H])
    ang.append(p.speaker); conf.append(p.speaker_conf); vadf.append(p.vad)
ang = np.array(ang); conf = np.array(conf); vadf = np.array(vadf)
skip = int(0.5 * fs / H)                      # ignore filter/noise-floor warm-up
v = vadf.copy(); v[:skip] = False
print(f"speech frames: {v.sum()}/{n} ({100*v.mean():.0f}%)   processing: {p.proc_ms:.2f} ms/hop (budget {1000*H/fs:.1f} ms)")
if v.sum() < 10:
    print("Too little speech detected -> speak louder/closer, or lower doa.vad_margin_db in config.json")
else:
    th = np.deg2rad(ang[v]); mean = np.rad2deg(np.arctan2(np.sin(th).mean(), np.cos(th).mean())) % 360
    spread = np.rad2deg(np.sqrt(-2*np.log(max(np.hypot(np.sin(th).mean(), np.cos(th).mean()), 1e-9))))
    print(f"DOA: mean {mean:.1f}°  circular std {spread:.1f}°  median conf {np.median(conf[v]):.2f}")
    print("pair TDOA (us): " + "  ".join(f"M{i+1}-M{j+1}:{t*1e6:6.1f}" for (i, j), t in zip(p.arr.pairs, p.doa.pair_tdoa())))
    if a.expect is not None:
        e = np.abs(angdiff(ang[v], a.expect)); print(f"error vs expected {a.expect:.0f}°: median {np.median(e):.1f}°  90th pct {np.percentile(e,90):.1f}°")
# input-vs-output level (speech frames only)
os.makedirs(os.path.dirname(a.out) or ".", exist_ok=True)
pk = np.abs(out).max()
with wave.open(a.out, "wb") as o:
    o.setnchannels(1); o.setsampwidth(2); o.setframerate(fs); o.writeframes((out * 32767).astype(np.int16).tobytes())
print(f"wrote {a.out}  (peak {pk:.2f})")
