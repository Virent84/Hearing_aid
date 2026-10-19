#!/usr/bin/env python3
"""Pretend to be the ESP32-S3: streams the exact packet format over a pseudo-terminal so the whole Pi chain
can be tested with no hardware.  Prints the device path (/dev/pts/N) to use as --port.
   python3 tools/fake_esp32.py --target 90 --shift 2 --drop 0.01 --claps"""
import argparse, os, sys, time, struct, binascii, copy
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from ha import config
from ha.audio_source import SimSource

ap = argparse.ArgumentParser()
ap.add_argument("--target", type=float, default=60); ap.add_argument("--interferer", type=float, default=200)
ap.add_argument("--shift", type=int, default=0, help="delay M3/M4 by this many samples (simulated I2S port offset)")
ap.add_argument("--drop", type=float, default=0.0, help="probability of dropping a packet")
ap.add_argument("--garbage", action="store_true", help="inject junk bytes between packets")
ap.add_argument("--claps", action="store_true", help="add a broadband clap from straight above every 1.5 s")
ap.add_argument("--seconds", type=float, default=1e9)
a = ap.parse_args()
cfg = config.deep_update(config.load(), {"sim": {"target_deg": a.target, "interferer_deg": a.interferer, "move": False}})
fs = cfg["sample_rate"]; src = SimSource(cfg, realtime=False); N = 128
m, s = os.openpty(); path = os.ttyname(s)
print(path, flush=True)
rng = np.random.default_rng(1); seq = 0; t = time.time(); sent = 0; tail = np.zeros((2, a.shift), np.float32)
while sent < a.seconds * fs:
    x = src.read(N)
    if a.claps:
        for k in range(N):
            ph = (src.t - N + k) % int(1.5 * fs)
            if ph < 6: x[:, k] += 0.6 * (rng.standard_normal() * (1 - ph / 6))
    if a.shift:
        z = np.concatenate([tail, x[2:4]], axis=1); x[2:4] = z[:, :N]; tail = z[:, N:]
    pcm = (np.clip(x, -1, 1) * 32767).astype("<i2").T.copy().tobytes()
    body = struct.pack("<HHH", seq & 0xFFFF, N, 0) + pcm
    pkt = b"\x5a\xa5" + body + struct.pack("<H", binascii.crc_hqx(body, 0xFFFF))
    seq += 1; sent += N
    if rng.random() >= a.drop:
        if a.garbage and rng.random() < 0.02: pkt = bytes(rng.integers(0, 256, 37, dtype=np.uint8)) + pkt
        try: os.write(m, pkt)
        except OSError: pass
    t += N / fs; d = t - time.time()
    if d > 0: time.sleep(d)
