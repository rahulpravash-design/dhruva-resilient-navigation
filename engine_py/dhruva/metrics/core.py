"""Outage / re-entry / spoof metrics. Definitions follow the master spec section 7.6."""
import numpy as np

from ..geo import to_enu

PS_DRIFT_PCT_MAX = 10.0
PS_ERR_PER_KM_MAX_M = 100.0


def _errors(est_lat, est_lon, truth_lat, truth_lon):
    e, n = to_enu(est_lat, est_lon, truth_lat, truth_lon)
    return np.hypot(e, n)


def outage_metrics(t, est_lat, est_lon, cov95_m, truth_lat, truth_lon, t0, t1):
    """Metrics over the outage window t0 <= t <= t1. Arrays are per-row and equal length."""
    t = np.asarray(t, float)
    w = (t >= t0) & (t <= t1)
    if w.sum() < 2:
        raise ValueError("outage_metrics: window contains fewer than 2 rows")
    tlat = np.asarray(truth_lat)[w]
    tlon = np.asarray(truth_lon)[w]
    err = _errors(np.asarray(est_lat)[w], np.asarray(est_lon)[w], tlat, tlon)
    te, tn = to_enu(tlat, tlon, tlat[0], tlon[0])
    dist = float(np.sum(np.hypot(np.diff(te), np.diff(tn))))
    endpoint = float(err[-1])
    cov = np.asarray(cov95_m, float)[w]
    coverage = float(np.mean(err <= cov) * 100) if np.all(np.isfinite(cov)) else float("nan")
    has_dist = dist > 0
    drift = endpoint / dist * 100 if has_dist else float("nan")
    per_km = endpoint * 1000 / dist if has_dist else float("nan")
    return {
        "endpoint_err_m": endpoint,
        "max_err_m": float(err.max()),
        "distance_m": dist,
        "drift_pct": drift,
        "err_per_km_m": per_km,
        "coverage_pct": coverage,
        "ps_pass": bool(has_dist and drift <= PS_DRIFT_PCT_MAX and per_km <= PS_ERR_PER_KM_MAX_M),
    }


def reentry_jump_m(t, disp_lat, disp_lon, speed, t_reacq, window_s=3.0):
    """Largest displayed-position step beyond expected motion (speed*dt) in [t_reacq, t_reacq+window_s]."""
    t = np.asarray(t, float)
    idx = np.flatnonzero((t >= t_reacq) & (t <= t_reacq + window_s))
    idx = idx[idx > 0]
    if idx.size == 0:
        return float("nan")
    lat, lon, spd = np.asarray(disp_lat), np.asarray(disp_lon), np.asarray(speed)
    e, n = to_enu(lat[idx], lon[idx], lat[idx - 1], lon[idx - 1])
    excess = np.hypot(e, n) - spd[idx] * (t[idx] - t[idx - 1])
    return float(max(0.0, excess.max()))


def time_to_detect_s(t, flag, t_event):
    """Seconds from t_event until flag is first True at or after it; NaN if never."""
    t = np.asarray(t, float)
    hit = np.flatnonzero(np.asarray(flag, bool) & (t >= t_event))
    return float(t[hit[0]] - t_event) if hit.size else float("nan")


def spoof_stats(detected, false_alarms, clean_duration_s):
    detected = np.asarray(detected, bool)
    per_hour = false_alarms / clean_duration_s * 3600 if clean_duration_s > 0 else float("nan")
    return {
        "detection_rate": float(detected.mean()) if detected.size else float("nan"),
        "false_alarms_per_hour": per_hour,
    }


def summarize(values):
    """Median, 90th percentile and n over finite values (NaN ignored)."""
    v = np.asarray(values, float)
    v = v[np.isfinite(v)]
    if v.size == 0:
        return {"median": float("nan"), "p90": float("nan"), "n": 0}
    return {"median": float(np.median(v)), "p90": float(np.percentile(v, 90)), "n": int(v.size)}
