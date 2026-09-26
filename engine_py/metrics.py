"""Evaluation metrics. Definitions: docs/CONTRACT.md."""
import numpy as np

from .geo import to_enu

CHI2_95_2DOF = 5.991


def evaluate(est, truth, mask=None):
    """est: output-contract columns; truth: dict with lat/lon per row.
    mask: optional bool array restricting the evaluated rows (e.g. an outage window)."""
    n = len(est["t"])
    mask = np.ones(n, bool) if mask is None else np.asarray(mask, bool)
    if mask.sum() < 2:
        raise ValueError("evaluate: mask selects fewer than 2 rows")

    lat0, lon0 = truth["lat"][0], truth["lon"][0]
    ee, en = to_enu(est["lat"], est["lon"], lat0, lon0)
    te, tn = to_enu(truth["lat"], truth["lon"], lat0, lon0)
    de, dn = ee - te, en - tn
    err = np.hypot(de, dn)

    idx = np.flatnonzero(mask)
    dist = float(np.sum(np.hypot(np.diff(te[idx]), np.diff(tn[idx]))))
    endpoint = float(err[idx[-1]])

    cee, cen, cnn = est["cov_ee"], est["cov_en"], est["cov_nn"]
    det = cee * cnn - cen ** 2
    with np.errstate(divide="ignore", invalid="ignore"):
        m2 = (cnn * de ** 2 - 2 * cen * de * dn + cee * dn ** 2) / det
    inside = (det > 0) & (m2 <= CHI2_95_2DOF)

    return {
        "endpoint_error_m": endpoint,
        "max_error_m": float(err[mask].max()),
        "drift_pct": endpoint / dist * 100 if dist > 0 else float("nan"),
        "coverage_95": float(inside[mask].mean()),
        "reentry_jump_m": _reentry_jump(est, ee, en, mask),
        "distance_m": dist,
        "n_rows": int(mask.sum()),
    }


def _reentry_jump(est, ee, en, mask):
    mode = np.asarray(est["mode"]).astype(str)
    jumps = []
    for i in range(1, len(mode)):
        if mode[i] == "GNSS" and mode[i - 1] != "GNSS" and mask[i - 1]:
            dt = est["t"][i] - est["t"][i - 1]
            h = np.radians(est["heading"][i - 1])
            step = est["speed"][i - 1] * dt
            pe = ee[i - 1] + step * np.sin(h)
            pn = en[i - 1] + step * np.cos(h)
            jumps.append(np.hypot(ee[i] - pe, en[i] - pn))
    return float(max(jumps)) if jumps else float("nan")
