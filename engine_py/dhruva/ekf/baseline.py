"""Ablation baseline: gyro-integrated heading + last GNSS speed held constant. No EKF, no covariance."""
import numpy as np

from ..geo import heading_deg_to_psi, wrap_pi
from .base import EngineBase


class BaselineEngine(EngineBase):
    def _init(self, t, fix):
        self.p = np.zeros(2)
        self.v = float(fix.get("speed", 0.0))
        bearing = fix.get("bearing", np.nan)
        self.psi = float(heading_deg_to_psi(bearing)) if np.isfinite(bearing) else 0.0

    def _propagate(self, f, wz, dt):
        self.psi = float(wrap_pi(self.psi + wz * dt))
        self.p += self.v * np.array([np.cos(self.psi), np.sin(self.psi)]) * dt

    def _on_fix(self, t, fix, e, n):
        self.p = np.array([e, n])
        self.v = float(fix["speed"])
        if self.v > self.cfg["ekf"]["gnss_vel_speed_min_mps"] and np.isfinite(fix.get("bearing", np.nan)):
            self.psi = float(heading_deg_to_psi(fix["bearing"]))

    def _pos_enu(self):
        return self.p[0], self.p[1]

    def _tick_updates(self, t):
        pass

    def _snapshot(self, t):
        lat, lon = self._pos_latlon(self.p[0], self.p[1], t)
        return {"t": float(t), "lat": lat, "lon": lon, "speed": self.v, "psi": self.psi,
                "cov95_m": float("nan"), "mode": self._mode(t), "gyro_bias": float("nan"),
                "nhc_active": False, "zupt_active": False, "gnss_trust": self._trust(t)}
