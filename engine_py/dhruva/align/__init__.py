"""Mount alignment. P2 ships only the oracle; the real estimator arrives in P3."""
import numpy as np


class OracleAligner:
    """Uses the simulator's true mount R_pv (phone <- vehicle). Reference for 'perfect mount' runs."""

    def __init__(self, R_pv):
        self.R = np.asarray(R_pv, float)

    def to_vehicle(self, acc, gyr):
        """Phone-frame accel/gyro -> (fx, fy, wz) in the vehicle frame (x fwd, y left, z up)."""
        f = self.R.T @ np.asarray(acc, float)
        w = self.R.T @ np.asarray(gyr, float)
        return float(f[0]), float(f[1]), float(w[2])
