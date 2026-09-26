"""Mount-agnostic alignment: phone frame -> levelled vehicle frame (x fwd, y left, z up).

Roll/pitch: gravity direction u (unit "up" in phone coords), propagated with the gyro and slowly corrected
by the accelerometer only when the vehicle is not accelerating or turning.
Yaw: least squares between the levelled horizontal specific force and the accelerations known from GNSS
speed and the gyro (a_long = d(speed)/dt, a_lat = speed * yaw_rate). With z_h = ax + i*ay in the level frame and
z_v = a_long + i*a_lat in the vehicle frame, z_h = exp(i*alpha) * z_v, so alpha = arg(sum z_h * conj(z_v)).
This also resolves the forward/backward sign. A PCA ratio on straight segments is kept as a diagnostic.
"""
from collections import deque

import numpy as np

_EX = np.array([1.0, 0.0, 0.0])
_EY = np.array([0.0, 1.0, 0.0])


def _unit(v):
    return v / np.linalg.norm(v)


class MountAligner:
    def __init__(self, cfg):
        self.g = cfg["gravity_mps2"]
        a = cfg["align"]
        self.a = a
        self.reset_all()

    def reset_all(self):
        self.u = None
        self._t = None
        self._t0 = None
        self._ref = _EX
        self._hist = deque()
        self._hold_until = -np.inf
        self._realigning = False
        self._h_fast = self._h_slow = None
        self._grav_hold_until = -np.inf
        self._gb = np.zeros(3)         # gyro bias estimate (phone frame), used for gravity propagation
        self._gb_n = 0
        self._speed = None            # (t, speed) latest GNSS speed
        self._along = 0.0             # GNSS d(speed)/dt [m/s^2]
        self._along_t = -np.inf
        self._vib = 0.0
        self._an_lp = None
        self._reset_yaw()

    def _reset_yaw(self):
        # Sums over gated samples: W, sum(h), sum(v), sum(h*conj(v)), sum(|v|^2). Means are removed when read,
        # so constant offsets (accel bias, gravity leak) drop out without any filter memory.
        self._W = 0.0
        self._Sh = 0j
        self._Sv = 0j
        self._Shv = 0j
        self._Svv = 0.0
        self._Wp = 0.0                 # straight-segment sums for the PCA-ratio diagnostic
        self._Shp = np.zeros(2)
        self._Cp = np.zeros((2, 2))

    # ---- inputs ---------------------------------------------------------------
    def on_gnss(self, t, speed):
        if self._speed is not None and t > self._speed[0]:
            self._along = (speed - self._speed[1]) / (t - self._speed[0])
            self._along_t = t
        self._speed = (t, float(speed))

    def on_imu(self, t, acc, gyr):
        acc = np.asarray(acc, float)
        gyr = np.asarray(gyr, float)
        a = self.a
        if self._t is None:
            self._t = self._t0 = t
            self.u = _unit(acc)
            self._an_lp = np.linalg.norm(acc)
            return
        dt = t - self._t
        if dt <= 0:
            return
        self._t = t

        if np.linalg.norm(gyr - self._gb) < a["gyro_bias_gate_radps"]:
            self._gb_n += 1
            self._gb += max(dt / a["gyro_bias_tau_s"], 1.0 / self._gb_n) * (gyr - self._gb)
        u = self.u - np.cross(gyr - self._gb, self.u) * dt
        an = float(np.linalg.norm(acc))
        quiet = t - self._along_t > 2.0 or abs(self._along) < a["gravity_gate_along_mps2"]
        init = t - self._t0 < a["gravity_init_s"]
        # Fast detector for the vehicle starting to accelerate/brake: horizontal accel leaves its slow average.
        horiz = acc - (acc @ self.u) * self.u
        if self._h_fast is None:
            self._h_fast = self._h_slow = horiz
        self._h_fast = self._h_fast + (dt / (a["gravity_dev_fast_tau_s"] + dt)) * (horiz - self._h_fast)
        self._h_slow = self._h_slow + (dt / (a["gravity_dev_slow_tau_s"] + dt)) * (self._h_fast - self._h_slow)
        if not init and np.linalg.norm(self._h_fast - self._h_slow) > a["gravity_gate_dev_mps2"]:
            self._grav_hold_until = t + a["gravity_hold_s"]
        # Without fresh GNSS the accel correction is frozen (gyro-only propagation): otherwise the tilt keeps
        # absorbing accelerometer bias while the EKF's own bias estimate is frozen, which drifts the DR.
        gnss_ok = self._speed is not None and t - self._speed[0] < a["gravity_gnss_fresh_s"]
        if abs(an - self.g) < a["gravity_gate_norm_mps2"] and np.linalg.norm(gyr) < a["gravity_gate_gyro_radps"] \
                and quiet and t >= self._grav_hold_until and (init or gnss_ok):
            tau = a["gravity_init_tau_s"] if init else a["gravity_tau_s"]
            u = u + (dt / tau) * (acc / an - u)
        self.u = _unit(u)

        # vibration energy (high-passed |a|) for RIGID/LOOSE
        k = dt / (0.5 + dt)
        self._an_lp += k * (an - self._an_lp)
        self._vib += (dt / (a["vibration_tau_s"] + dt)) * ((an - self._an_lp) ** 2 - self._vib)

        self._check_jolt(t)
        self._yaw_step(t, dt, acc, gyr)

    # ---- internals ---------------------------------------------------------------
    def _level(self):
        u = self.u
        if abs(self._ref @ u) > 0.9:
            self._ref = _EY if self._ref is _EX else _EX
            self._reset_yaw()      # level-frame x axis changed: yaw estimate no longer valid
        x = _unit(self._ref - (self._ref @ u) * u)
        return np.array([x, np.cross(u, x), u])

    def _check_jolt(self, t):
        h = self._hist
        if not h or t - h[-1][0] >= 0.1:
            h.append((t, self.u.copy()))
        while h and t - h[0][0] > 1.0:
            h.popleft()
        if t < self._hold_until or len(h) < 2:
            return
        ang = np.degrees(np.arccos(np.clip(self.u @ h[0][1], -1.0, 1.0)))
        if ang > self.a["realign_gravity_change_deg"]:
            self._reset_yaw()
            self._realigning = True
            self._hold_until = t + self.a["realign_hold_s"]

    def _yaw_step(self, t, dt, acc, gyr):
        a = self.a
        a_l = self._level() @ acc
        z_h = complex(a_l[0], a_l[1])
        w_zl = float(gyr @ self.u)
        fresh = self._speed is not None and t - self._speed[0] < 2.0
        if t < self._hold_until or not fresh or self._speed[1] < a["yaw_speed_min_mps"]:
            return
        v = self._speed[1]
        straight = abs(w_zl) < a["straight_gyro_z_max_radps"]
        if straight:
            z_v = complex(self._along, 0.0)                     # longitudinal reference only
        elif abs(self._along) < a["yaw_along_gate_mps2"]:
            z_v = complex(0.0, v * w_zl)                        # lateral reference only (no braking/accelerating)
        else:
            return                                              # braking into a turn: the two mix, skip

        lam = a["forget_per_s"] ** dt
        self._W = lam * self._W + dt
        self._Sh = lam * self._Sh + z_h * dt
        self._Sv = lam * self._Sv + z_v * dt
        self._Shv = lam * self._Shv + z_h * np.conj(z_v) * dt
        self._Svv = lam * self._Svv + abs(z_v) ** 2 * dt
        if straight and v > a["straight_speed_min_mps"]:
            hv = np.array([z_h.real, z_h.imag])
            self._Wp = lam * self._Wp + dt
            self._Shp = lam * self._Shp + hv * dt
            self._Cp = lam * self._Cp + np.outer(hv, hv) * dt
        if self._realigning and self.confidence >= a["realign_done_confidence"]:
            self._realigning = False

    # ---- outputs ---------------------------------------------------------------
    def _cov(self):
        """Mean-removed correlation sum and excitation energy of the yaw evidence."""
        if self._W <= 0:
            return 0j, 0.0
        S = self._Shv - self._Sh * np.conj(self._Sv) / self._W
        E = self._Svv - abs(self._Sv) ** 2 / self._W
        return S, max(E, 0.0)

    @property
    def alpha(self):
        S = self._cov()[0]
        return float(np.angle(S)) if S != 0 else 0.0

    @property
    def confidence(self):
        return float(min(1.0, self._cov()[1] / self.a["min_excitation_m2ps3"]))

    @property
    def confident(self):
        return self.confidence >= self.a["confident_threshold"] and not self._realigning

    @property
    def realigning(self):
        return self._realigning

    @property
    def pca_ratio(self):
        if self._Wp <= 0:
            return float("nan")
        lam = np.linalg.eigvalsh(self._Cp - np.outer(self._Shp, self._Shp) / self._Wp)
        return float(lam[1] / max(lam[0], 1e-12))

    @property
    def mount_class(self):
        return "LOOSE" if np.sqrt(max(self._vib, 0.0)) > self.a["loose_vibration_rms_mps2"] else "RIGID"

    @property
    def R_vp(self):
        """Vehicle <- phone rotation (3x3)."""
        c, s = np.cos(self.alpha), np.sin(self.alpha)
        Rz = np.array([[c, s, 0.0], [-s, c, 0.0], [0.0, 0.0, 1.0]])   # Rz(-alpha)
        return Rz @ self._level()

    def to_vehicle(self, acc, gyr):
        """Phone-frame accel/gyro -> (fx, fy, wz) in the vehicle frame."""
        f = self.R_vp @ np.asarray(acc, float)
        return float(f[0]), float(f[1]), float(np.asarray(gyr, float) @ self.u)


def mount_errors_deg(R_vp_est, R_pv_true):
    """(tilt_error, yaw_error) in degrees between an estimated vehicle<-phone rotation and the true one."""
    R_true = np.asarray(R_pv_true, float).T
    up_e, up_t = R_vp_est[2], R_true[2]
    tilt = np.degrees(np.arccos(np.clip(up_e @ up_t, -1.0, 1.0)))

    def horiz(fwd):
        f = fwd - (fwd @ up_t) * up_t
        return f / np.linalg.norm(f)

    fe, ft = horiz(R_vp_est[0]), horiz(R_true[0])
    yaw = np.degrees(np.arccos(np.clip(fe @ ft, -1.0, 1.0)))
    return float(tilt), float(yaw)
