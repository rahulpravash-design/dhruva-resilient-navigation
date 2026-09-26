"""Fault injectors on a fixture DataFrame. Each returns a modified copy."""
import numpy as np

from ..geo import offset_latlon

_FIX_COLS = ("gnss_lat", "gnss_lon", "gnss_acc", "gnss_speed", "gnss_bearing", "cn0_mean", "sats_used")


def apply_outage(df, t_start, dur_s):
    """No GNSS for t_start <= t < t_start + dur_s: no fixes, gnss_valid False."""
    out = df.copy()
    win = (out["t"] >= t_start) & (out["t"] < t_start + dur_s)
    out.loc[win, "gnss_new"] = False
    out.loc[win, "gnss_valid"] = False
    out.loc[win, list(_FIX_COLS)] = np.nan
    return out


def _shift(out, t_start, dist_of_t, bearing_deg):
    m = out["gnss_new"] & (out["t"] >= t_start)
    lat, lon = offset_latlon(out.loc[m, "gnss_lat"].to_numpy(), out.loc[m, "gnss_lon"].to_numpy(),
                             dist_of_t(out.loc[m, "t"].to_numpy()), bearing_deg)
    out.loc[m, "gnss_lat"] = lat
    out.loc[m, "gnss_lon"] = lon
    return out


def apply_spoof_step(df, t_start, offset_m=500.0, bearing_deg=90.0):
    """From t_start on, every fix is displaced by a constant offset (sudden jump)."""
    return _shift(df.copy(), t_start, lambda t: offset_m, bearing_deg)


def apply_spoof_ramp(df, t_start, rate_mps=2.0, bearing_deg=90.0):
    """From t_start on, fixes are displaced by an offset growing at rate_mps (slow drift)."""
    return _shift(df.copy(), t_start, lambda t: rate_mps * (t - t_start), bearing_deg)
