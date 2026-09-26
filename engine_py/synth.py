"""Synthetic drive generator. Everything it produces is SIMULATED, not evidence.

Phone flat, x forward, y left, z up. Heading is clockwise from north, so
heading rate = -gz.
"""
import numpy as np

from .geo import to_latlon

G = 9.81
T0 = 1.7e9  # arbitrary epoch start


def make_drive(duration=400.0, imu_hz=10, gnss_hz=1, seed=0, origin=(28.6139, 77.2090)):
    """Return (trace, truth). trace follows the input contract; truth has lat/lon per row."""
    rng = np.random.default_rng(seed)
    dt = 1.0 / imu_hz
    n = int(duration * imu_hz) + 1
    tau = np.arange(n) * dt
    t = T0 + tau

    w = 2 * np.pi / 60.0
    v = 12.0 + 2.0 * np.sin(w * tau)          # m/s
    a_long = 2.0 * w * np.cos(w * tau)        # m/s^2

    yaw = 0.05 * np.sin(2 * np.pi * tau / 40.0)  # rad/s, CCW positive
    for t_start, t_end, rate in ((100, 110, 0.15), (200, 208, -0.2), (300, 310, 0.12)):
        yaw = np.where((tau >= t_start) & (tau < t_end), rate, yaw)

    theta = np.empty(n)
    theta[0] = np.radians(45.0)
    theta[1:] = theta[0] - np.cumsum(yaw[:-1]) * dt
    e = np.zeros(n)
    nn = np.zeros(n)
    e[1:] = np.cumsum(v[:-1] * np.sin(theta[:-1]) * dt)
    nn[1:] = np.cumsum(v[:-1] * np.cos(theta[:-1]) * dt)
    tlat, tlon = to_latlon(e, nn, *origin)

    trace = {
        "t": t,
        "ax": a_long + 0.02 + rng.normal(0, 0.05, n),
        "ay": v * yaw + rng.normal(0, 0.05, n),
        "az": G + rng.normal(0, 0.05, n),
        "gx": rng.normal(0, 0.005, n),
        "gy": rng.normal(0, 0.005, n),
        "gz": yaw + 0.002 + rng.normal(0, 0.005, n),
        "mx": np.full(n, np.nan), "my": np.full(n, np.nan), "mz": np.full(n, np.nan),
    }

    fix = np.zeros(n, bool)
    fix[:: max(1, int(round(imu_hz / gnss_hz)))] = True
    nan = np.full(n, np.nan)
    ge = e + rng.normal(0, 3.0, n)
    gn = nn + rng.normal(0, 3.0, n)
    glat, glon = to_latlon(ge, gn, *origin)
    trace["gnss_lat"] = np.where(fix, glat, nan)
    trace["gnss_lon"] = np.where(fix, glon, nan)
    trace["gnss_acc"] = np.where(fix, 4.0, nan)
    trace["gnss_speed"] = np.where(fix, np.maximum(0, v + rng.normal(0, 0.2, n)), nan)
    trace["gnss_bearing"] = np.where(fix, np.degrees(theta + rng.normal(0, np.radians(2), n)) % 360, nan)
    trace["cn0_mean"] = np.where(fix, 38 + rng.normal(0, 1.5, n), nan)
    trace["n_sats"] = np.where(fix, 14.0, nan)
    trace["gnss_valid"] = fix

    return trace, {"lat": tlat, "lon": tlon}
