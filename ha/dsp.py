"""Core DSP: preprocessing -> GCC-PHAT -> DOA (SRP-PHAT) -> beamforming."""
import numpy as np
from scipy import signal
from .geometry import Array


class Preprocessor:
    """DC removal + high-pass (stateful), slow per-mic sensitivity equalisation."""
    def __init__(self, cfg, M):
        p = cfg["preprocess"]; fs = cfg["sample_rate"]
        self.sos = signal.butter(2, p["highpass_hz"], "highpass", fs=fs, output="sos")
        self.zi = np.zeros((self.sos.shape[0], M, 2))
        self.eq = p["equalize_mics"]; self.gain = np.ones(M); self.rms = np.full(M, 1e-4)

    def __call__(self, x, active=True):
        y, self.zi = signal.sosfilt(self.sos, x, axis=1, zi=self.zi)
        if self.eq and active:
            r = np.sqrt(np.mean(y ** 2, axis=1)) + 1e-9
            self.rms = 0.995 * self.rms + 0.005 * r
            g = self.rms.mean() / self.rms
            self.gain = np.clip(g, 0.7, 1.4)
        return y * self.gain[:, None]


class NoiseReducer:
    """Min-statistics noise PSD + Wiener-style gain; same real gain on all mics (keeps phase)."""
    def __init__(self, cfg, F):
        self.floor = 10 ** (cfg["preprocess"]["nr_floor_db"] / 20)
        self.on = cfg["preprocess"]["noise_reduction"]
        self.sm = np.full(F, 1e-8); self.hist = np.full((96, F), 1e-8); self.k = 0
        self.noise = np.full(F, 1e-8); self.G = np.ones(F)

    def __call__(self, X):
        p = np.mean(np.abs(X) ** 2, axis=0)
        self.sm = 0.85 * self.sm + 0.15 * p
        self.hist[self.k % len(self.hist)] = self.sm; self.k += 1
        self.noise = self.hist.min(axis=0) * 1.5
        snr = np.maximum(self.sm / (self.noise + 1e-12) - 1, 0)
        g = snr / (snr + 1)
        self.G = 0.6 * self.G + 0.4 * np.maximum(np.sqrt(g), self.floor)
        return X * self.G[None] if self.on else X


class GccDoa:
    """GCC-PHAT cross-spectra per mic pair (recursively averaged) + SRP-PHAT scan -> DOA."""
    def __init__(self, cfg, arr: Array, F, fs, nfft):
        d = cfg["doa"]; self.arr = arr; self.fs = fs; self.nfft = nfft
        f = np.fft.rfftfreq(nfft, 1 / fs); self.f = f
        self.band = (f >= d["band_hz"][0]) & (f <= d["band_hz"][1])
        self.fb = f[self.band]
        step = d["angle_step_deg"]
        self.grid = np.arange(0, 360, step); th = np.deg2rad(self.grid)
        tau = arr.pair_tau(th)                                    # [A,P]
        self.steer = np.exp(2j * np.pi * tau[:, :, None] * self.fb[None, None, :])  # [A,P,Fb]
        self.G = np.zeros((len(arr.pairs), self.band.sum()), complex)
        self.alpha = d["smoothing"]; self.score = np.zeros(len(self.grid))
        self.max_lag = arr.pos.max() * 2 / arr.c

    def update(self, X, weight=1.0):
        Xb = X[:, self.band]
        for p, (i, j) in enumerate(self.arr.pairs):
            c = Xb[i] * np.conj(Xb[j]); c /= (np.abs(c) + 1e-12)   # PHAT weighting
            self.G[p] = self.alpha * self.G[p] + (1 - self.alpha) * c
        return self

    def scan(self):
        s = np.einsum("apf,pf->a", self.steer, self.G).real / (self.G.shape[0] * self.G.shape[1])
        self.score = s
        k = int(np.argmax(s))
        # parabolic refinement
        y0, y1, y2 = s[k - 1], s[k], s[(k + 1) % len(s)]
        den = y0 - 2 * y1 + y2
        off = 0.5 * (y0 - y2) / den if abs(den) > 1e-12 else 0
        ang = (self.grid[k] + off * (self.grid[1] - self.grid[0])) % 360
        # confidence: peak prominence over mean
        conf = float(np.clip(s[k] - np.mean(s), 0, 1))
        return float(ang), conf

    def pair_tdoa(self, interp=8):
        """Per-pair TDOA (seconds) from the averaged GCC-PHAT, with sinc interpolation."""
        out = []
        n = self.nfft * interp
        for p in range(self.G.shape[0]):
            S = np.zeros(self.nfft // 2 + 1, complex); S[self.band] = self.G[p]
            cc = np.fft.irfft(S, n); cc = np.fft.fftshift(cc)
            lags = (np.arange(n) - n // 2) / (self.fs * interp)
            m = np.abs(lags) <= self.max_lag * 1.2
            out.append(float(lags[m][np.argmax(cc[m])]))
        return out


class Beamformer:
    """Frequency-domain steered beamformer: delay-and-sum or MVDR (diagonal loaded)."""
    def __init__(self, cfg, arr: Array, F, fs, nfft):
        b = cfg["beamformer"]; self.arr = arr; self.method = b["method"]
        self.load = b["diag_loading"]; self.a = b["cov_smoothing"]
        self.f = np.fft.rfftfreq(nfft, 1 / fs)
        self.R = np.tile(np.eye(arr.M, dtype=complex), (F, 1, 1)) * 1e-6
        self.theta = None; self.d = None

    def steering(self, theta_deg):
        t = self.arr.arrival_times(np.deg2rad(theta_deg))               # [M]
        return np.exp(-2j * np.pi * self.f[:, None] * t[None, :])       # [F,M]  d(f)

    def __call__(self, X, theta_deg, adapt=True):
        Xt = X.T                                                        # [F,M]
        if adapt:
            self.R = self.a * self.R + (1 - self.a) * (Xt[:, :, None] * np.conj(Xt[:, None, :]))
        d = self.steering(theta_deg); M = self.arr.M
        if self.method == "mvdr":
            tr = np.real(np.trace(self.R, axis1=1, axis2=2))[:, None, None] / M
            Rl = self.R + (self.load * tr + 1e-10) * np.eye(M)[None]
            Ri_d = np.linalg.solve(Rl, d[:, :, None])[:, :, 0]
            w = Ri_d / (np.sum(np.conj(d) * Ri_d, axis=1, keepdims=True) + 1e-12)
        else:
            w = d / M
        return np.sum(np.conj(w) * Xt, axis=1)                          # [F]


class STFT:
    def __init__(self, frame, hop, M):
        self.N, self.H = frame, hop
        self.win = np.sqrt(signal.windows.hann(frame, sym=False))
        self.buf = np.zeros((M, frame)); self.ola = np.zeros(frame)

    def analyze(self, hop_samples):
        self.buf = np.concatenate([self.buf[:, self.H:], hop_samples], axis=1)
        return np.fft.rfft(self.buf * self.win[None], axis=1)

    def synth(self, Y):
        y = np.fft.irfft(Y, self.N) * self.win
        self.ola += y
        out = self.ola[: self.H].copy()
        self.ola = np.concatenate([self.ola[self.H:], np.zeros(self.H)])
        return out
