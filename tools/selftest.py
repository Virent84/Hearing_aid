"""Offline validation of DSP chain on synthetic data (no hardware, no real-time pacing)."""
import sys, os, copy, numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from ha import config
from ha.audio_source import SimSource
from ha.pipeline import Pipeline, angdiff

def run(cfg, secs=14):
    cfg = copy.deepcopy(cfg); src = SimSource(cfg, realtime=False); p = Pipeline(cfg, src)
    H = cfg["stft"]["hop"]; n = int(secs * cfg["sample_rate"] / H); errs = []; sir_in = []; sir_out = []
    ys = []; ts = []
    for k in range(n):
        x = src.read(H); y = p.step(x)
        if k > 3 * cfg["sample_rate"] / H and p.vad:
            errs.append(abs(angdiff(p.speaker, src.truth)))
    return np.array(errs)

if __name__ == "__main__":
    base = config.load(); ok = True
    for tgt, intf in [(60, 200), (150, 300), (270, 30), (10, 100)]:
        for method in ("das", "mvdr"):
            c = config.deep_update(base, {"sim": {"target_deg": tgt, "interferer_deg": intf, "move": False, "snr_db": 15},
                                          "beamformer": {"method": method}})
            e = run(c, 10)
            med = np.median(e) if len(e) else 999
            print(f"target {tgt:3d}°  interferer {intf:3d}°  {method:4s}: median DOA err {med:5.1f}°  (frames w/ speech: {len(e)})")
            ok &= med < 15
    print("PASS" if ok else "FAIL"); sys.exit(0 if ok else 1)
