"""GNSS receiver model: 1 Hz fixes with position/speed/bearing noise, C/N0 and satellite count."""
from dataclasses import dataclass

import numpy as np

from ..geo import psi_to_heading_deg, to_latlon


@dataclass(frozen=True)
class GnssParams:
    rate_hz: float = 1.0
    pos_noise_std_m: float = 3.5   # per axis
    speed_noise_std: float = 0.1
    bearing_noise_std_deg: float = 3.0
    cn0_mean: float = 38.0
    cn0_std: float = 2.0
    sats_mean: float = 12.0


def gnss_columns(traj, origin, params, rng, imu_hz=100):
    """Per-row GNSS columns. Fields are NaN on rows without a fresh fix (gnss_new False)."""
    n = len(traj["t"])
    new = np.zeros(n, bool)
    new[:: round(imu_hz / params.rate_hz)] = True
    nan = np.full(n, np.nan)

    ge = traj["e"] + rng.normal(0, params.pos_noise_std_m, n)
    gn = traj["n"] + rng.normal(0, params.pos_noise_std_m, n)
    lat, lon = to_latlon(ge, gn, *origin)
    speed = np.maximum(0.0, traj["v"] + rng.normal(0, params.speed_noise_std, n))
    bearing = (psi_to_heading_deg(traj["psi"]) + rng.normal(0, params.bearing_noise_std_deg, n)) % 360
    bearing = np.where(traj["v"] >= 1.0, bearing, np.nan)
    sats = np.clip(np.round(rng.normal(params.sats_mean, 1.5, n)), 5, 20)

    return {
        "gnss_new": new,
        "gnss_lat": np.where(new, lat, nan),
        "gnss_lon": np.where(new, lon, nan),
        "gnss_acc": np.where(new, 1.2 * params.pos_noise_std_m, nan),
        "gnss_speed": np.where(new, speed, nan),
        "gnss_bearing": np.where(new, bearing, nan),
        "cn0_mean": np.where(new, params.cn0_mean + rng.normal(0, params.cn0_std, n), nan),
        "sats_used": np.where(new, sats, nan),
        "gnss_valid": np.ones(n, bool),
    }
