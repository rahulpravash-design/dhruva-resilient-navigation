"""Mount alignment: the real estimator (MountAligner) and the simulator-truth oracle used as a reference."""
import numpy as np

from .mount import MountAligner, mount_errors_deg


class OracleAligner:
    """Uses the simulator's true mount R_pv (phone <- vehicle). Reference for 'perfect mount' runs."""

    confident = True
    realigning = False

    def __init__(self, R_pv):
        self.R = np.asarray(R_pv, float)

    def on_imu(self, t, acc, gyr):
        pass

    def on_gnss(self, t, speed):
        pass

    def to_vehicle_full(self, acc, gyr):
        return self.R.T @ np.asarray(acc, float), self.R.T @ np.asarray(gyr, float)

    def to_vehicle(self, acc, gyr):
        """Phone-frame accel/gyro -> (fx, fy, wz) in the vehicle frame (x fwd, y left, z up)."""
        f = self.R.T @ np.asarray(acc, float)
        w = self.R.T @ np.asarray(gyr, float)
        return float(f[0]), float(f[1]), float(w[2])


__all__ = ["MountAligner", "OracleAligner", "mount_errors_deg"]
