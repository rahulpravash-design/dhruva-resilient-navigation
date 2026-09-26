"""Engine I/O contract validators. Definition of record: docs/CONTRACT.md."""
import numpy as np

INPUT_FIELDS = (
    "t", "ax", "ay", "az", "gx", "gy", "gz", "mx", "my", "mz",
    "gnss_lat", "gnss_lon", "gnss_acc", "gnss_speed", "gnss_bearing",
    "cn0_mean", "n_sats", "gnss_valid",
)
OUTPUT_FIELDS = (
    "t", "lat", "lon", "cov_ee", "cov_en", "cov_nn",
    "speed", "heading", "mode", "gnss_trust",
)
MODES = ("GNSS", "DR", "DEGRADED")


class ContractError(ValueError):
    pass


def _length(trace, fields, name):
    missing = [f for f in fields if f not in trace]
    if missing:
        raise ContractError(f"{name}: missing fields {missing}")
    n = len(trace["t"])
    bad = [f for f in fields if len(trace[f]) != n]
    if bad:
        raise ContractError(f"{name}: fields {bad} differ in length from t ({n})")
    if n == 0:
        raise ContractError(f"{name}: empty trace")
    return n


def _need(cond, msg):
    if not np.all(cond):
        raise ContractError(msg)


def _check_time(t, name):
    _need(np.isfinite(t), f"{name}: t must be finite")
    _need(np.diff(t) > 0, f"{name}: t must be strictly increasing")


def validate_input(trace):
    _length(trace, INPUT_FIELDS, "input")
    t = np.asarray(trace["t"], float)
    _check_time(t, "input")
    for f in ("ax", "ay", "az", "gx", "gy", "gz"):
        _need(np.isfinite(np.asarray(trace[f], float)), f"input: {f} must be finite")
    valid = np.asarray(trace["gnss_valid"])
    if valid.dtype != bool:
        raise ContractError("input: gnss_valid must be bool")
    if not valid[0]:
        raise ContractError("input: first row must carry a GNSS fix")
    lat = np.asarray(trace["gnss_lat"], float)
    lon = np.asarray(trace["gnss_lon"], float)
    _need(np.isnan(lat[~valid]) & np.isnan(lon[~valid]),
          "input: gnss_lat/gnss_lon must be NaN where gnss_valid is False")
    v = valid
    _need(np.isfinite(lat[v]) & (np.abs(lat[v]) <= 90), "input: gnss_lat invalid on fix rows")
    _need(np.isfinite(lon[v]) & (np.abs(lon[v]) <= 180), "input: gnss_lon invalid on fix rows")
    acc = np.asarray(trace["gnss_acc"], float)[v]
    _need(np.isfinite(acc) & (acc > 0), "input: gnss_acc must be finite and > 0 on fix rows")
    spd = np.asarray(trace["gnss_speed"], float)[v]
    _need(np.isfinite(spd) & (spd >= 0), "input: gnss_speed must be finite and >= 0 on fix rows")
    for f in ("cn0_mean", "n_sats"):
        x = np.asarray(trace[f], float)[v]
        _need(np.isfinite(x) & (x >= 0), f"input: {f} must be finite and >= 0 on fix rows")
    brg = np.asarray(trace["gnss_bearing"], float)[v]
    brg = brg[np.isfinite(brg)]
    _need((brg >= 0) & (brg < 360), "input: gnss_bearing must be in [0, 360) or NaN")


def validate_output(out, n_expected=None):
    n = _length(out, OUTPUT_FIELDS, "output")
    if n_expected is not None and n != n_expected:
        raise ContractError(f"output: {n} rows, expected {n_expected}")
    _check_time(np.asarray(out["t"], float), "output")
    lat = np.asarray(out["lat"], float)
    lon = np.asarray(out["lon"], float)
    _need(np.isfinite(lat) & (np.abs(lat) <= 90), "output: lat invalid")
    _need(np.isfinite(lon) & (np.abs(lon) <= 180), "output: lon invalid")
    cee, cen, cnn = (np.asarray(out[k], float) for k in ("cov_ee", "cov_en", "cov_nn"))
    _need(np.isfinite(cee) & np.isfinite(cen) & np.isfinite(cnn), "output: covariance must be finite")
    _need((cee >= 0) & (cnn >= 0), "output: cov_ee and cov_nn must be >= 0")
    _need(cee * cnn - cen ** 2 >= -1e-9 * (1 + cee * cnn), "output: covariance not positive semi-definite")
    spd = np.asarray(out["speed"], float)
    _need(np.isfinite(spd) & (spd >= 0), "output: speed must be finite and >= 0")
    hdg = np.asarray(out["heading"], float)
    _need(np.isfinite(hdg) & (hdg >= 0) & (hdg < 360), "output: heading must be in [0, 360)")
    bad = set(np.unique(np.asarray(out["mode"]).astype(str))) - set(MODES)
    if bad:
        raise ContractError(f"output: unknown mode(s) {sorted(bad)}")
    trust = np.asarray(out["gnss_trust"], float)
    _need(np.isfinite(trust) & (trust >= 0) & (trust <= 1), "output: gnss_trust must be in [0, 1]")
