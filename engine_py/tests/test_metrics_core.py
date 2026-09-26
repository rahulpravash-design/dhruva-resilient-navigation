import numpy as np
import pytest

from dhruva.geo import to_latlon
from dhruva.metrics import outage_metrics, reentry_jump_m, spoof_stats, summarize, time_to_detect_s

LAT0, LON0 = 28.6, 77.2
T = np.arange(0.0, 101.0)                       # 1 Hz, 100 s
TRUTH = to_latlon(np.zeros_like(T), 10.0 * T, LAT0, LON0)   # north at 10 m/s -> 1000 m


def _est(east_offset_m):
    return to_latlon(np.full_like(T, east_offset_m), 10.0 * T, LAT0, LON0)


def test_hand_computed_outage_metrics():
    est = _est(30.0)
    m = outage_metrics(T, *est, np.full_like(T, 40.0), *TRUTH, 0, 100)
    assert m["endpoint_err_m"] == pytest.approx(30.0, abs=0.01)
    assert m["max_err_m"] == pytest.approx(30.0, abs=0.01)
    assert m["distance_m"] == pytest.approx(1000.0, abs=0.01)
    assert m["drift_pct"] == pytest.approx(3.0, abs=0.001)
    assert m["err_per_km_m"] == pytest.approx(30.0, abs=0.01)
    assert m["coverage_pct"] == 100.0
    assert m["ps_pass"] is True


def test_window_and_coverage():
    est = _est(30.0)
    m = outage_metrics(T, *est, np.full_like(T, 20.0), *TRUTH, 20, 60)
    assert m["distance_m"] == pytest.approx(400.0, abs=0.01)
    assert m["coverage_pct"] == 0.0
    assert m["drift_pct"] == pytest.approx(7.5, abs=0.001)


def test_ps_pass_boundary():
    ones = np.ones_like(T)
    assert outage_metrics(T, *_est(99.9), ones, *TRUTH, 0, 100)["ps_pass"] is True
    assert outage_metrics(T, *_est(100.1), ones, *TRUTH, 0, 100)["ps_pass"] is False


def test_window_too_small():
    with pytest.raises(ValueError):
        outage_metrics(T, *_est(1.0), np.ones_like(T), *TRUTH, 5.0, 5.2)


def test_reentry_jump_excess_over_expected_motion():
    n = 10.0 * T
    n[50:] += 8.0                               # 8 m extra step at t=50
    lat, lon = to_latlon(np.zeros_like(T), n, LAT0, LON0)
    speed = np.full_like(T, 10.0)
    assert reentry_jump_m(T, lat, lon, speed, 49.0) == pytest.approx(8.0, abs=0.01)
    assert reentry_jump_m(T, lat, lon, speed, 60.0) == pytest.approx(0.0, abs=1e-6)
    assert np.isnan(reentry_jump_m(T, lat, lon, speed, 500.0))


def test_time_to_detect():
    t = np.arange(10.0)
    flag = t >= 5
    assert time_to_detect_s(t, flag, 3.0) == 2.0
    assert np.isnan(time_to_detect_s(t, np.zeros(10, bool), 3.0))
    assert np.isnan(time_to_detect_s(t, t < 2, 3.0))


def test_spoof_stats():
    s = spoof_stats([True, True, False, True], false_alarms=2, clean_duration_s=1800)
    assert s["detection_rate"] == 0.75 and s["false_alarms_per_hour"] == 4.0


def test_summarize_ignores_nan():
    s = summarize([1, 2, 3, 4, 100, float("nan")])
    assert s["n"] == 5 and s["median"] == 3.0 and s["p90"] == pytest.approx(61.6)
    assert summarize([])["n"] == 0
