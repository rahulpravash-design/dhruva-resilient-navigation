"""Shared engine plumbing: timing, ticks, forced outages, spoof injection, integrity gate, stationary detection."""
from collections import deque
from dataclasses import dataclass

import numpy as np
import pandas as pd

from ..geo import heading_deg_to_psi, offset_latlon, to_enu, to_latlon
from ..integrity import APPLY, FixInfo, IntegrityMonitor, Prediction
from ..speednet.features import WindowBuffer, feature_row

TIMELINE_COLUMNS = ("t", "lat", "lon", "speed", "psi", "cov95_m", "mode", "gyro_bias", "nhc_active", "zupt_active",
                    "gnss_trust")
MODE_SLACK_S = 0.05  # scheduling slack so a fix arriving exactly 1.0 s after the last one is not a gap
HISTORY_S = 20.0


@dataclass(frozen=True)
class EngineFlags:
    use_nhc: bool = True
    use_zupt: bool = True                 # ZUPT + ZARU
    use_ml: bool = False                  # SpeedNet speed pseudo-measurement (needs a speed_estimator)
    use_integrity: bool = False           # GNSS trust guard (spoof rejection, safe re-entry, 2 s display blend)
    force_outage: tuple = ()              # ((t_start, dur_s), ...) fixes dropped inside these windows
    spoof: tuple = None                   # ("step", t_start, metres) or ("ramp", t_start, m/s): offsets fixes to the east


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
    gnss_trust: float


class EngineBase:
    supports_integrity = False

    def __init__(self, config, flags=None, aligner=None, speed_estimator=None):
        self.cfg = config
        self.flags = flags or EngineFlags()
        if self.flags.use_ml and speed_estimator is None:
            raise ValueError("use_ml requires a speed_estimator")
        self.aligner = aligner
        self.speed_estimator = speed_estimator
        self.monitor = IntegrityMonitor(config) if self.flags.use_integrity and self.supports_integrity else None
        self._feat = WindowBuffer(config["speednet"]["window_samples"])
        self.origin = None
        self.t = None
        self._last_fix_t = -np.inf          # any fix seen (used to veto ZUPT)
        self._last_fix_speed = 0.0
        self._last_applied_t = -np.inf      # last fix that drove the filter (used for mode)
        self._tick_dt = 1.0 / config["ekf"]["tick_hz"]
        self._next_tick = None
        self._win = deque()
        self._hist = deque()                # (t, E, N) of the filter position, for consistency checks
        self._blend_off = np.zeros(2)
        self._blend_t0 = -np.inf
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
        fix = self._spoofed(t, fix)
        if np.isfinite(fix.get("speed", np.nan)):
            self.aligner.on_gnss(t, fix["speed"])
        if self.origin is None:
            self.origin = (fix["lat"], fix["lon"])
            self.t = t
            self._init(t, fix)
            self._last_fix_t = self._last_applied_t = t
            self._last_fix_speed = float(fix.get("speed", 0.0))
            if self.monitor:
                self.monitor.on_seen(t)
            self._record(t)
            self._next_tick = t + self._tick_dt
            return
        e, n = to_enu(fix["lat"], fix["lon"], *self.origin)
        e, n = float(e), float(n)
        self._last_fix_t, self._last_fix_speed = t, float(fix.get("speed", 0.0))
        shown_before = None
        if self.monitor:
            if not self._integrity_accepts(t, fix, e, n):
                return
            if self.monitor.reentered:      # remember what was on screen, so the correction can be eased in
                shown_before = np.array(self._pos_enu()) + self._blend_off * self._blend_factor(t)
        self._on_fix(t, fix, e, n)
        self._last_applied_t = t
        if self.monitor:
            self.monitor.on_seen(t)
        if shown_before is not None:
            self._blend_off = shown_before - np.array(self._pos_enu())
            self._blend_t0 = t

    def state(self):
        return EngineState(**self._snapshot(self.t))

    def timeline_df(self):
        return pd.DataFrame(self._timeline, columns=TIMELINE_COLUMNS)

    def log_df(self):
        return pd.DataFrame(self._log, columns=("t", "kind", "nis", "dof"))

    def events_df(self):
        ev = self.monitor.events if self.monitor else []
        return pd.DataFrame(ev, columns=("t", "kind", "detail"))

    # ---- integrity ------------------------------------------------------------
    def _spoofed(self, t, fix):
        sp = self.flags.spoof
        if not sp or t < sp[1]:
            return fix
        dist = sp[2] if sp[0] == "step" else sp[2] * (t - sp[1])
        lat, lon = offset_latlon(fix["lat"], fix["lon"], dist, 90.0)
        return {**fix, "lat": float(lat), "lon": float(lon)}

    def _disp_fn(self):
        h = self._hist
        if len(h) < 2:
            return None
        ts = np.array([r[0] for r in h])
        es = np.array([r[1] for r in h])
        ns = np.array([r[2] for r in h])
        return lambda a, b: (float(np.interp(b, ts, es) - np.interp(a, ts, es)),
                             float(np.interp(b, ts, ns) - np.interp(a, ts, ns)))

    def _integrity_accepts(self, t, fix, e, n):
        pred = self._predict_fix(fix, e, n)
        bearing = fix.get("bearing", np.nan)
        info = FixInfo(t, e, n, float(fix["acc"]), float(fix.get("sats", 12.0)), float(fix.get("speed", 0.0)),
                       float(heading_deg_to_psi(bearing)) if np.isfinite(bearing) else float("nan"))
        decision = self.monitor.on_fix(info, Prediction(pred["nis"], pred["cov95_m"], pred["speed"], pred["psi"],
                                                        self._disp_fn()))
        return decision == APPLY

    def _blend_factor(self, t):
        """1 at re-entry, easing to 0 over `display_blend_s`: the displayed position never jumps."""
        return max(0.0, 1.0 - (t - self._blend_t0) / self.cfg["integrity"]["display_blend_s"])

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
        if self.monitor:
            base = self.monitor.mode
            gnss = base == "GNSS"
        else:
            gnss = t - self._last_applied_t <= self.cfg["integrity"]["no_fix_timeout_s"] + MODE_SLACK_S
            base = "GNSS" if gnss else "DR"
        if self.aligner.realigning or (base == "DR" and not self.aligner.confident):
            return "DEGRADED"
        return base

    def _trust(self, t):
        if self.monitor:
            return self.monitor.trust
        return 1.0 if self._mode(t) == "GNSS" else 0.0

    def _pos_latlon(self, e, n, t):
        off = self._blend_off * self._blend_factor(t)
        lat, lon = to_latlon(e + off[0], n + off[1], *self.origin)
        return float(lat), float(lon)

    def _record(self, t):
        row = self._snapshot(t)
        self._timeline.append([row[c] for c in TIMELINE_COLUMNS])

    def _ml_window(self):
        return self._feat.window() if self.flags.use_ml and self._feat.full else None

    def _tick(self, t):
        self._nhc = self._zupt = False
        if self.monitor:
            self.monitor.on_tick(t)
        self._tick_updates(t)
        e, n = self._pos_enu()
        self._hist.append((t, float(e), float(n)))
        while t - self._hist[0][0] > HISTORY_S:
            self._hist.popleft()
        self._record(t)

    # ---- hooks --------------------------------------------------------------
    def _init(self, t, fix):
        raise NotImplementedError

    def _propagate(self, f, wz, dt):
        raise NotImplementedError

    def _on_fix(self, t, fix, e, n):
        raise NotImplementedError

    def _predict_fix(self, fix, e, n):
        raise NotImplementedError

    def _pos_enu(self):
        raise NotImplementedError

    def _tick_updates(self, t):
        raise NotImplementedError

    def _snapshot(self, t):
        raise NotImplementedError
