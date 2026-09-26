"""Planar error-state EKF. State x = [pE, pN, vE, vN, psi, b_g, b_ax, b_ay]; psi is CCW from East."""
import numpy as np

from ..geo import wrap_pi

_J = np.array([[0.0, -1.0], [1.0, 0.0]])
_I8 = np.eye(8)


class PlanarEkf:
    def __init__(self, x0, P0, accel_noise, gyro_noise, gyro_bias_rw, accel_bias_rw):
        self.x = np.array(x0, float)
        self.P = np.array(P0, float)
        self.q = (accel_noise, gyro_noise, gyro_bias_rw, accel_bias_rw)

    def predict(self, f, wz, dt):
        """f: levelled vehicle-frame horizontal specific force (fx, fy); wz: yaw rate [rad/s]."""
        x = self.x
        c, s = np.cos(x[4]), np.sin(x[4])
        R = np.array([[c, -s], [s, c]])
        a = R @ (np.asarray(f, float) - x[6:8])

        F = np.zeros((8, 8))
        F[0, 2] = F[1, 3] = 1.0
        F[2:4, 4] = _J @ a
        F[2:4, 6:8] = -R
        F[4, 5] = -1.0
        Phi = _I8 + F * dt + 0.5 * (F @ F) * dt * dt

        x[0:2] += x[2:4] * dt + 0.5 * a * dt * dt
        x[2:4] += a * dt
        x[4] = float(wrap_pi(x[4] + (wz - x[5]) * dt))

        qa, qg, qbg, qba = self.q
        Q = np.zeros((8, 8))
        Q[2, 2] = Q[3, 3] = qa * qa * dt
        Q[4, 4] = qg * qg * dt
        Q[5, 5] = qbg * qbg * dt
        Q[6, 6] = Q[7, 7] = qba * qba * dt
        self.P = Phi @ self.P @ Phi.T + Q

    def update(self, z, hx, H, R):
        """Joseph-form update. Returns the NIS of this update."""
        y = np.atleast_1d(np.asarray(z, float) - hx)
        H = np.atleast_2d(H)
        R = np.atleast_2d(R)
        P = self.P
        S = H @ P @ H.T + R
        K = np.linalg.solve(S, H @ P).T
        self.x += K @ y
        self.x[4] = float(wrap_pi(self.x[4]))
        IKH = _I8 - K @ H
        P = IKH @ P @ IKH.T + K @ R @ K.T
        self.P = 0.5 * (P + P.T)
        return float(y @ np.linalg.solve(S, y))

    def cov95_m(self):
        a, b, d = self.P[0, 0], self.P[0, 1], self.P[1, 1]
        lam = 0.5 * (a + d) + np.sqrt((0.5 * (a - d)) ** 2 + b * b)
        return float(np.sqrt(5.991 * max(lam, 0.0)))
