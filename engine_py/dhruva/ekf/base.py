"""Shared engine plumbing: timing, ticks, forced outages, stationary detection, timeline."""
from collections import deque
from dataclasses import dataclass

import numpy as np
import pandas as pd

from ..geo import to_enu, to_latlon

TIMELINE_COLUMNS = ("t", "lat", "lon", "speed", "psi", "cov95_m", "mode", "gyro_bias", "nhc_active", "zupt_active")
MODE_SLACK_S = 0.05  # scheduling slack so a fix arriving exactly 1.0 s after the last one is not a gap


@dataclass(frozen=True)
class EngineFlags:
    use_nhc: bool = True
    use_zupt: bool = True                 # ZUPT + ZARU
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
    def __init__(self, config, flags=None, aligner=None):
        self.cfg = config
        self.flags = flags or EngineFlags()
        self.aligner = aligner
        self.origin = None
        self.t = None
        self._last_fix_t = -np.inf
        self._tick_dt = 1.0 / config["ekf"]["tick_hz"]
        self._next_tick = None
        self._win = deque()
        self._timeline = []
        self._log = []
        self._nhc = False
        self._zupt = False

    # ---- public interface -------------------------------------------------
    def on_imu(self, t, acc, gyr):
        if self.origin is None:
            return
        fx, fy, wz = self.aligner.to_vehicle(acc, gyr)
        dt = t - self.t
        if dt > 0:
            self._propagate((fx, fy), wz, dt)
            self.t = t
        self._push_window(t, acc, gyr, wz)
        while t >= self._next_tick - 1e-9:
            self._tick(t)
            self._next_tick += self._tick_dt

    def on_gnss(self, t, fix):
        if any(t0 <= t < t0 + d for t0, d in self.flags.force_outage):
            return
        if self.origin is None:
            self.origin = (fix["lat"], fix["lon"])
            self.t = t
            self._init(t, fix)
            self._last_fix_t = t
            self._record(t)
            self._next_tick = t + self._tick_dt
            return
        e, n = to_enu(fix["lat"], fix["lon"], *self.origin)
        self._on_fix(t, fix, float(e), float(n))
        self._last_fix_t = t

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
        self._win.append((t, float(np.linalg.norm(gyr)), abs(float(np.linalg.norm(acc)) - g), wz))
        while t - self._win[0][0] > w:
            self._win.popleft()

    def _stationary(self):
        qs = self.cfg["align"]["quasi_static"]
        if not self._win or self._win[-1][0] - self._win[0][0] < 0.99 * qs["window_s"]:
            return False
        return all(g < qs["gyro_norm_max_radps"] and a < qs["accel_norm_dev_max_mps2"]
                   for _, g, a, _ in self._win)

    def _mode(self, t):
        timeout = self.cfg["integrity"]["no_fix_timeout_s"] + MODE_SLACK_S
        return "GNSS" if t - self._last_fix_t <= timeout else "DR"

    def _pos_latlon(self, e, n):
        lat, lon = to_latlon(e, n, *self.origin)
        return float(lat), float(lon)

    def _record(self, t):
        row = self._snapshot(t)
        self._timeline.append([row[c] for c in TIMELINE_COLUMNS])

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
