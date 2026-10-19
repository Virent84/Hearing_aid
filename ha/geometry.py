import numpy as np

class Array:
    """Angles: degrees, 0 = +x axis, counter-clockwise. Far-field plane-wave model."""
    def __init__(self, cfg):
        a = cfg["array"]
        self.c = a["speed_of_sound"]
        self.angles = np.deg2rad(np.array(a["mic_angles_deg"], float))
        self.pos = a["radius_m"] * np.stack([np.cos(self.angles), np.sin(self.angles)], 1)  # [M,2]
        self.M = len(self.angles)
        self.pairs = [(i, j) for i in range(self.M) for j in range(i + 1, self.M)]

    def arrival_times(self, theta_rad):
        """t_i (relative) for source direction theta; closer mic hears earlier. shape [..., M]"""
        u = np.stack([np.cos(theta_rad), np.sin(theta_rad)], -1)
        return -(u @ self.pos.T) / self.c

    def pair_tau(self, theta_rad):
        t = self.arrival_times(theta_rad)
        return np.stack([t[..., i] - t[..., j] for i, j in self.pairs], -1)  # [..., P]
