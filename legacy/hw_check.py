#!/usr/bin/env python3
"""Hardware bring-up: records a few seconds from the I2S card and prints per-channel level.
Tap/talk near ONE mic at a time to confirm which channel is which mic.
   python3 tools/hw_check.py [-D hw:0,0] [-c 8] [-r 16000]"""
import argparse, subprocess, sys, time
import numpy as np
ap = argparse.ArgumentParser()
ap.add_argument("-D", default=None); ap.add_argument("-c", type=int, default=8)
ap.add_argument("-r", type=int, default=16000); ap.add_argument("-s", type=float, default=0.5)
a = ap.parse_args()
print(subprocess.run(["arecord", "-l"], capture_output=True, text=True).stdout or "no capture cards found")
if a.D is None:
    sys.path.insert(0, __file__.rsplit("/", 2)[0]); from ha.audio_source import ALSASource
    a.D = ALSASource._find_device()
print(f"device {a.D}, {a.c} ch @ {a.r} Hz  (Ctrl+C to stop)\n")
p = subprocess.Popen(["arecord", "-q", "-D", a.D, "-c", str(a.c), "-r", str(a.r), "-f", "S32_LE", "-t", "raw"], stdout=subprocess.PIPE)
n = int(a.r * a.s) * a.c * 4
try:
    while True:
        b = p.stdout.read(n)
        if len(b) < n: print("stream ended - check overlay / format / channel count"); break
        x = np.frombuffer(b, "<i4").reshape(-1, a.c).T / 2**31
        rms = np.sqrt(np.mean(x ** 2, axis=1)); db = 20 * np.log10(rms + 1e-9)
        dc = np.mean(x, axis=1)
        print(" | ".join(f"ch{c}:{db[c]:6.1f}dB{'*' if rms[c] > 1e-7 else ' '}" for c in range(a.c)), end="\r", flush=True)
except KeyboardInterrupt:
    p.kill(); print("\n'*' = live channel. Expect 4 live channels (0,2,4,6 for 8ch or 0-3 for 4ch). Set alsa.channel_map in config.json if needed.")
