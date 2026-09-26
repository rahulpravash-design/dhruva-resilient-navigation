"""Shared engine plumbing: timing, ticks, forced outages, stationary detection, timeline."""
from collections import deque
from dataclasses import dataclass

import numpy as np
import pandas as pd

from ..geo import to_enu, to_latlon
from ..speednet.features import WindowBuffer, feature_row

TIMELINE_COLUMNS = ("t", "lat", "lon", "speed", "psi", "cov95_m", "mode", "gyro_bias", "nhc_active", "zupt_active")
MODE_SLACK_S = 0.05  # scheduling slack so a fix arriving exactly 1.0 s after the last one is not a gap


@dataclass(frozen=True)
class EngineFlags:
    use_nhc: bool = True
    use_zupt: bool = True                 # ZUPT + ZARU
    use_ml: bool = False                  # SpeedNet speed pseudo-measurement (needs a speed_estimator)
    force_outage: tuple = ()              # ((t_start, dur_s), ...) fixes dropped inside these windows


@dataclass
class EngineState:
    t: float
    lat: float
    lon: float
    speed: float
    psi: float
    cov95_m: float
    mode: str
    gyro_bias: float
    nhc_active: bool
    zupt_active: bool


class EngineBase:
    def __init__(self, config, flags=None, aligner=None, speed_estimator=None):
        self.cfg = config
        self.flags = flags or EngineFlags()
        if self.flags.use_ml and speed_estimator is None:
            raise ValueError("use_ml requires a speed_estimator")
        self.aligner = aligner
        self.speed_estimator = speed_estimator
        self._feat = WindowBuffer(config["speednet"]["window_samples"])
        self.origin = None
        self.t = None
        self._last_fix_t = -np.inf
        self._last_fix_speed = 0.0
        self._tick_dt = 1.0 / config["ekf"]["tick_hz"]
        self._next_tick = None
        self._win = deque()
        self._timeline = []
        self._log = []
        self._nhc = False
        self._zupt = False

    # ---- public interface -------------------------------------------------
    def on_imu(self, t, acc, gyr):
        self.aligner.on_imu(t, acc, gyr)
        if self.origin is None:
            return
        fx, fy, wz = self.aligner.to_vehicle(acc, gyr)
        dt = t - self.t
        if dt > 0:
            self._propagate((fx, fy) if self.aligner.confident else None, wz, dt)
            self.t = t
        self._push_window(t, acc, gyr, wz)
        if self.flags.use_ml:
            if self.aligner.confident:
                a_v, w_v = self.aligner.to_vehicle_full(acc, gyr)
                self._feat.push(feature_row(a_v, w_v, self.cfg["gravity_mps2"]))
            else:
                self._feat.clear()          # levelled frame not trustworthy: restart the window
        while t >= self._next_tick - 1e-9:
            self._tick(t)
            self._next_tick += self._tick_dt

    def on_gnss(self, t, fix):
        if any(t0 <= t < t0 + d for t0, d in self.flags.force_outage):
            return
        if np.isfinite(fix.get("speed", np.nan)):
            self.aligner.on_gnss(t, fix["speed"])
        if self.origin is None:
            self.origin = (fix["lat"], fix["lon"])
            self.t = t
            self._init(t, fix)
            self._last_fix_t = t
            self._last_fix_speed = float(fix.get("speed", 0.0))
            self._record(t)
            self._next_tick = t + self._tick_dt
            return
        e, n = to_enu(fix["lat"], fix["lon"], *self.origin)
        self._on_fix(t, fix, float(e), float(n))
        self._last_fix_t = t
        self._last_fix_speed = float(fix.get("speed", 0.0))

    def state(self):
        row = self._snapshot(self.t)
        return EngineState(**row)

    def timeline_df(self):
        return pd.DataFrame(self._timeline, columns=TIMELINE_COLUMNS)

    def log_df(self):
        return pd.DataFrame(self._log, columns=("t", "kind", "nis", "dof"))

    # ---- helpers ------------------------------------------------------------
    def _push_window(self, t, acc, gyr, wz):
        g = self.cfg["gravity_mps2"]
        w = self.cfg["align"]["quasi_static"]["window_s"]
        self._win.append((t, float(np.linalg.norm(gyr)), abs(float(np.linalg.norm(acc)) - g), wz,
                          np.asarray(acc, float)))
        while t - self._win[0][0] > w:
            self._win.popleft()

    def _stationary(self, t):
        """Quasi-static IMU over the window AND no evidence of motion.

        The accel norm barely changes when a vehicle accelerates horizontally, so the accel *vector* must also be
        steady, and a fresh GNSS fix reporting movement vetoes it. Without these, ZUPT clamps speed to zero while the
        vehicle is pulling away from a stop and the accel-bias states absorb the real acceleration.
        """
        qs = self.cfg["align"]["quasi_static"]
        k = self.cfg["ekf"]
        if not self._win or self._win[-1][0] - self._win[0][0] < 0.99 * qs["window_s"]:
            return False
        if not all(g < qs["gyro_norm_max_radps"] and a < qs["accel_norm_dev_max_mps2"] for _, g, a, _, _ in self._win):
            return False
        if t - self._last_fix_t < k["zupt_gnss_veto_age_s"] and self._last_fix_speed > k["zupt_gnss_veto_speed_mps"]:
            return False
        acc = np.array([w[4] for w in self._win])
        return float(np.max(np.linalg.norm(acc - acc.mean(axis=0), axis=1))) < qs["accel_vec_dev_max_mps2"]

    def _mode(self, t):
        timeout = self.cfg["integrity"]["no_fix_timeout_s"] + MODE_SLACK_S
        gnss = t - self._last_fix_t <= timeout
        if self.aligner.realigning or (not gnss and not self.aligner.confident):
            return "DEGRADED"
        return "GNSS" if gnss else "DR"

    def _pos_latlon(self, e, n):
        lat, lon = to_latlon(e, n, *self.origin)
        return float(lat), float(lon)

    def _record(self, t):
        row = self._snapshot(t)
        self._timeline.append([row[c] for c in TIMELINE_COLUMNS])

    def _ml_window(self):
        return self._feat.window() if self.flags.use_ml and self._feat.full else None

    def _tick(self, t):
        self._nhc = self._zupt = False
        self._tick_updates(t)
        self._record(t)

    # ---- hooks --------------------------------------------------------------
    def _init(self, t, fix):
        raise NotImplementedError

    def _propagate(self, f, wz, dt):
        raise NotImplementedError

    def _on_fix(self, t, fix, e, n):
        raise NotImplementedError

    def _tick_updates(self, t):
        raise NotImplementedError

    def _snapshot(self, t):
        raise NotImplementedError
