"""SpeedNet input features: levelled vehicle-frame [ax, ay, az-g, gx, gy, gz, |a_horizontal|] over a 1.5 s window."""
from collections import deque

import numpy as np

G = 9.80665
WINDOW = 150
N_FEATURES = 7


def feature_rows(a_veh, w_veh, g=G):
    """a_veh, w_veh: (n, 3) vehicle-frame accel and gyro -> (n, 7) features."""
    a_veh, w_veh = np.asarray(a_veh, float), np.asarray(w_veh, float)
    ah = np.hypot(a_veh[:, 0], a_veh[:, 1])
    return np.column_stack([a_veh[:, 0], a_veh[:, 1], a_veh[:, 2] - g, w_veh[:, 0], w_veh[:, 1], w_veh[:, 2], ah])


def feature_row(a_veh, w_veh, g=G):
    return feature_rows(np.asarray(a_veh)[None], np.asarray(w_veh)[None], g)[0]


class WindowBuffer:
    """Most recent WINDOW feature rows at 100 Hz."""

    def __init__(self, size=WINDOW):
        self.size = size
        self._rows = deque(maxlen=size)

    def push(self, row):
        self._rows.append(row)

    def clear(self):
        self._rows.clear()

    @property
    def full(self):
        return len(self._rows) == self.size

    def window(self):
        return np.asarray(self._rows, dtype=np.float32)
