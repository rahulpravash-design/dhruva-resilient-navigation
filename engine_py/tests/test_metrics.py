import numpy as np
import pytest

from engine_py.geo import to_latlon
from engine_py.metrics import evaluate

LAT0, LON0 = 28.6, 77.2


def _path(e, n):
    lat, lon = to_latlon(e, n, LAT0, LON0)
    return {"lat": lat, "lon": lon}


def _est(e, n, sigma=5.0, mode=None):
    k = len(e)
    p = _path(e, n)
    p.update({
        "t": np.arange(k, dtype=float),
        "cov_ee": np.full(k, sigma ** 2), "cov_en": np.zeros(k), "cov_nn": np.full(k, sigma ** 2),
        "speed": np.zeros(k), "heading": np.zeros(k),
        "mode": np.array(mode if mode is not None else ["GNSS"] * k),
    })
    return p


def test_endpoint_max_and_drift():
    k = 101
    n = np.linspace(0, 100, k)            # truth: 100 m north
    e_err = np.zeros(k)
    e_err[-1] = 3.0                       # 3 m east error at the end only
    truth = _path(np.zeros(k), n)
    m = evaluate(_est(e_err, n), truth)
    assert m["endpoint_error_m"] == pytest.approx(3.0, abs=1e-3)
    assert m["max_error_m"] == pytest.approx(3.0, abs=1e-3)
    assert m["distance_m"] == pytest.approx(100.0, abs=1e-3)
    assert m["drift_pct"] == pytest.approx(3.0, abs=1e-2)


def test_mask_restricts_window():
    k = 101
    n = np.linspace(0, 100, k)
    e_err = np.zeros(k)
    e_err[-1] = 50.0                      # big error outside the window
    truth = _path(np.zeros(k), n)
    mask = np.zeros(k, bool)
    mask[10:51] = True
    m = evaluate(_est(e_err, n), truth, mask=mask)
    assert m["max_error_m"] == pytest.approx(0.0, abs=1e-3)
    assert m["distance_m"] == pytest.approx(40.0, abs=1e-3)


def test_coverage_extremes():
    k = 50
    n = np.linspace(0, 49, k)
    truth = _path(np.zeros(k), n)
    off = _est(np.full(k, 10.0), n)       # constant 10 m error
    assert evaluate({**off, "cov_ee": np.full(k, 1e4), "cov_nn": np.full(k, 1e4)}, truth)["coverage_95"] == 1.0
    assert evaluate({**off, "cov_ee": np.full(k, 1.0), "cov_nn": np.full(k, 1.0)}, truth)["coverage_95"] == 0.0


def test_coverage_boundary_is_chi2():
    k = 10
    n = np.zeros(k)
    truth = _path(np.zeros(k), n)
    sigma = 2.0
    inside = np.sqrt(5.991) * sigma * 0.99
    outside = np.sqrt(5.991) * sigma * 1.01
    assert evaluate(_est(np.full(k, inside), n, sigma), truth)["coverage_95"] == 1.0
    assert evaluate(_est(np.full(k, outside), n, sigma), truth)["coverage_95"] == 0.0


def test_reentry_jump():
    k = 6
    n = np.zeros(k)
    e = np.array([0, 0, 0, 0, 25.0, 25.0])   # stands still, then jumps 25 m on GNSS re-entry
    est = _est(e, n, mode=["GNSS", "DR", "DR", "DR", "GNSS", "GNSS"])
    m = evaluate(est, _path(np.zeros(k), n))  # speed 0 -> predicted position is unchanged
    assert m["reentry_jump_m"] == pytest.approx(25.0, abs=1e-3)


def test_no_reentry_is_nan():
    k = 5
    est = _est(np.zeros(k), np.zeros(k))
    assert np.isnan(evaluate(est, _path(np.zeros(k), np.zeros(k)))["reentry_jump_m"])
