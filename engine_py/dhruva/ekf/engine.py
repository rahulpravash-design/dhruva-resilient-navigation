"""EKF engine: GNSS position/velocity, NHC, ZUPT, ZARU on top of the planar error-state EKF."""
import numpy as np

from ..geo import heading_deg_to_psi
from .base import EngineBase
from .filter import PlanarEkf


class Engine(EngineBase):
    def _init(self, t, fix):
        k = self.cfg["ekf"]
        ini = k["init"]
        bearing = fix.get("bearing", np.nan)
        speed = fix.get("speed", 0.0)
        x0 = np.zeros(8)
        P0 = np.diag([ini["pos_sigma_m"] ** 2] * 2 + [ini["vel_sigma_mps"] ** 2] * 2
                     + [ini["psi_sigma_rad"] ** 2, ini["gyro_bias_sigma_radps"] ** 2]
                     + [ini["accel_bias_sigma_mps2"] ** 2] * 2)
        if np.isfinite(bearing) and speed >= k["gnss_vel_speed_min_mps"]:
            psi = float(heading_deg_to_psi(bearing))
            x0[2:4] = speed * np.array([np.cos(psi), np.sin(psi)])
            x0[4] = psi
        else:
            P0[4, 4] = ini["no_bearing_psi_sigma_rad"] ** 2
            P0[2, 2] = P0[3, 3] = max(speed, 1.0) ** 2
        self.ekf = PlanarEkf(x0, P0, k["accel_noise_mps2_rthz"], k["gyro_noise_radps_rthz"],
                             k["gyro_bias_rw_radps_rts"], k["accel_bias_rw_mps2_rts"],
                             k["no_accel_noise_mps2_rthz"])

    def _propagate(self, f, wz, dt):
        self.ekf.predict(f, wz, dt)

    def _update(self, t, kind, z, hx, H, R):
        nis = self.ekf.update(z, hx, H, R)
        self._log.append([t, kind, nis, int(np.size(z))])

    def _on_fix(self, t, fix, e, n):
        k = self.cfg["ekf"]
        sigma = max(fix["acc"], k["gnss_pos_sigma_floor_m"])
        H = np.zeros((2, 8))
        H[0, 0] = H[1, 1] = 1.0
        x = self.ekf.x
        self._update(t, "gnss_pos", [e, n], x[0:2], H, np.eye(2) * sigma ** 2)

        bearing, speed = fix.get("bearing", np.nan), fix.get("speed", 0.0)
        if speed > k["gnss_vel_speed_min_mps"] and np.isfinite(bearing):
            psi_m = float(heading_deg_to_psi(bearing))
            u = np.array([np.cos(psi_m), np.sin(psi_m)])
            up = np.array([-u[1], u[0]])
            s_al = k["gnss_vel_along_sigma_mps"]
            s_cr = max(s_al, speed * np.radians(k["gnss_bearing_sigma_deg"]))
            R = s_al ** 2 * np.outer(u, u) + s_cr ** 2 * np.outer(up, up)
            Hv = np.zeros((2, 8))
            Hv[0, 2] = Hv[1, 3] = 1.0
            self._update(t, "gnss_vel", speed * u, self.ekf.x[2:4], Hv, R)

    def _tick_updates(self, t):
        k = self.cfg["ekf"]
        x = self.ekf.x
        # Constant-velocity motion looks quasi-static to an IMU, so also require a low estimated speed.
        if self._stationary() and np.hypot(x[2], x[3]) < k["zupt_speed_max_mps"]:
            if self.flags.use_zupt:
                Hz = np.zeros((2, 8))
                Hz[0, 2] = Hz[1, 3] = 1.0
                self._update(t, "zupt", [0.0, 0.0], x[2:4], Hz, np.eye(2) * k["zupt_sigma_mps"] ** 2)
                Hb = np.zeros((1, 8))
                Hb[0, 5] = 1.0
                wz_mean = float(np.mean([w[3] for w in self._win]))
                self._update(t, "zaru", [wz_mean], [x[5]], Hb, np.array([[k["zaru_sigma_radps"] ** 2]]))
                self._zupt = True
        elif self.flags.use_nhc:
            c, s = np.cos(x[4]), np.sin(x[4])
            Hn = np.zeros((1, 8))
            Hn[0, 2], Hn[0, 3], Hn[0, 4] = -s, c, -(c * x[2] + s * x[3])
            self._update(t, "nhc", [0.0], [-s * x[2] + c * x[3]], Hn,
                         np.array([[k["nhc_sigma_mps"] ** 2]]))
            self._nhc = True

    def _snapshot(self, t):
        x = self.ekf.x
        lat, lon = self._pos_latlon(x[0], x[1])
        return {"t": float(t), "lat": lat, "lon": lon, "speed": float(np.hypot(x[2], x[3])),
                "psi": float(x[4]), "cov95_m": self.ekf.cov95_m(), "mode": self._mode(t),
                "gyro_bias": float(x[5]), "nhc_active": self._nhc, "zupt_active": self._zupt}
