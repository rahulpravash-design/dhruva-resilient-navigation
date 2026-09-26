import numpy as np
import pytest

from engine_py.contract import validate_input
from engine_py.geo import to_enu
from engine_py.inject import outage, spoof_drift, spoof_offset
from engine_py.synth import make_drive


@pytest.fixture
def trace():
    return make_drive(duration=200.0)[0]


def _shift(a, b, i):
    e, n = to_enu(a["gnss_lat"][i], a["gnss_lon"][i], b["gnss_lat"][i], b["gnss_lon"][i])
    return float(np.hypot(e, n))


def test_outage_window_only(trace):
    start = trace["t"][0] + 50
    out = outage(trace, start, 30)
    win = (trace["t"] >= start) & (trace["t"] < start + 30)
    assert not out["gnss_valid"][win].any()
    assert np.isnan(out["gnss_lat"][win]).all() and np.isnan(out["gnss_lon"][win]).all()
    np.testing.assert_array_equal(out["gnss_valid"][~win], trace["gnss_valid"][~win])
    validate_input(out)


def test_inputs_not_mutated(trace):
    before = {k: v.copy() for k, v in trace.items()}
    t0 = trace["t"][0]
    outage(trace, t0 + 10, 10)
    spoof_offset(trace, t0 + 10)
    spoof_drift(trace, t0 + 10)
    for k in trace:
        np.testing.assert_array_equal(trace[k], before[k])


def test_spoof_offset_is_500m_after_start_only(trace):
    start = trace["t"][0] + 100
    out = spoof_offset(trace, start, 500.0)
    valid = trace["gnss_valid"]
    late = np.flatnonzero(valid & (trace["t"] >= start))
    early = np.flatnonzero(valid & (trace["t"] < start))
    assert _shift(out, trace, late[0]) == pytest.approx(500.0, abs=0.5)
    assert _shift(out, trace, late[-1]) == pytest.approx(500.0, abs=0.5)
    assert _shift(out, trace, early[-1]) == pytest.approx(0.0, abs=1e-6)
    validate_input(out)


def test_spoof_drift_grows_at_rate(trace):
    start = trace["t"][0] + 100
    out = spoof_drift(trace, start, rate_mps=2.0)
    late = np.flatnonzero(trace["gnss_valid"] & (trace["t"] >= start))
    for i in (late[0], late[len(late) // 2], late[-1]):
        expected = 2.0 * (trace["t"][i] - start)
        assert _shift(out, trace, i) == pytest.approx(expected, abs=0.5)
    validate_input(out)
