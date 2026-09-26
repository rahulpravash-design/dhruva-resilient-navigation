import numpy as np

from dhruva.legacy.baseline import run
from dhruva.legacy.inject import outage
from dhruva.legacy.metrics import evaluate
from dhruva.legacy.synth import make_drive


def test_with_gnss_error_is_gnss_noise_level():
    trace, truth = make_drive(duration=200.0)
    m = evaluate(run(trace), truth)
    assert m["max_error_m"] < 20.0          # 3 m/axis GNSS noise plus <1 s of dead reckoning


def test_outage_makes_error_grow_and_sets_mode():
    trace, truth = make_drive(duration=300.0)
    start = trace["t"][0] + 100
    est = run(outage(trace, start, 60))
    win = (trace["t"] >= start) & (trace["t"] < start + 60)
    assert (est["mode"][win][30:] == "DR").all()
    m_out = evaluate(est, truth, mask=win)
    m_all_gnss = evaluate(run(trace), truth, mask=win)
    assert m_out["max_error_m"] > 2 * m_all_gnss["max_error_m"]


def test_reentry_returns_to_gnss():
    trace, truth = make_drive(duration=300.0)
    start = trace["t"][0] + 100
    est = run(outage(trace, start, 30))
    assert est["mode"][-1] == "GNSS"
    assert np.isfinite(evaluate(est, truth, mask=(trace["t"] < start + 30) & (trace["t"] >= start))["reentry_jump_m"])
