"""Fault injectors. Each returns a modified copy; the input trace is never mutated."""
import numpy as np

from .geo import offset_latlon

_GNSS_COLS = ("gnss_lat", "gnss_lon", "gnss_acc", "gnss_speed", "gnss_bearing", "cn0_mean", "n_sats")


def _copy(trace):
    return {k: np.array(v, copy=True) for k, v in trace.items()}


def outage(trace, start_t, duration_s):
    """Drop GNSS for rows with start_t <= t < start_t + duration_s."""
    out = _copy(trace)
    win = (out["t"] >= start_t) & (out["t"] < start_t + duration_s)
    out["gnss_valid"][win] = False
    for c in _GNSS_COLS:
        out[c][win] = np.nan
    return out


def spoof_offset(trace, start_t, offset_m=500.0, bearing_deg=90.0):
    """From start_t on, shift every GNSS fix by a constant offset (sudden jump)."""
    out = _copy(trace)
    m = out["gnss_valid"] & (out["t"] >= start_t)
    out["gnss_lat"][m], out["gnss_lon"][m] = offset_latlon(
        out["gnss_lat"][m], out["gnss_lon"][m], offset_m, bearing_deg)
    return out


def spoof_drift(trace, start_t, rate_mps=2.0, bearing_deg=90.0):
    """From start_t on, shift GNSS fixes by an offset growing at rate_mps (slow drift)."""
    out = _copy(trace)
    m = out["gnss_valid"] & (out["t"] >= start_t)
    dist = rate_mps * (out["t"][m] - start_t)
    out["gnss_lat"][m], out["gnss_lon"][m] = offset_latlon(
        out["gnss_lat"][m], out["gnss_lon"][m], dist, bearing_deg)
    return out
