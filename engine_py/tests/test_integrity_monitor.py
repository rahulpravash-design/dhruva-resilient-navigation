"""IntegrityMonitor state machine with synthetic predictions (no EKF)."""
import numpy as np
import pytest

from dhruva.config import load_config
from dhruva.integrity import APPLY, WITHHOLD, FixInfo, IntegrityMonitor, Prediction

CFG = load_config()


def fix(t, e=0.0, n=0.0, acc=4.0, sats=12, speed=10.0, psi=0.0):
    return FixInfo(t, e, n, acc, sats, speed, psi)


def pred(nis=1.0, cov=5.0, speed=10.0, psi=0.0, disp=None):
    return Prediction(nis, cov, speed, psi, disp)


def straight_disp(v=10.0):
    return lambda a, b: (v * (b - a), 0.0)


def test_normal_fixes_are_applied():
    m = IntegrityMonitor(CFG)
    assert all(m.on_fix(fix(t), pred(1.5)) == APPLY for t in range(20))
    assert m.state == "LOCK" and m.mode == "GNSS" and m.trust > 0.9


def test_isolated_outlier_is_withheld_but_not_a_spoof():
    m = IntegrityMonitor(CFG)
    assert m.on_fix(fix(0), pred(50.0)) == WITHHOLD
    assert m.on_fix(fix(1), pred(1.0)) == APPLY
    assert m.state == "LOCK"


def test_step_spoof_two_consecutive_outliers():
    m = IntegrityMonitor(CFG)
    assert m.on_fix(fix(0), pred(80.0, cov=6.0)) == WITHHOLD
    assert m.state == "LOCK"
    assert m.on_fix(fix(1), pred(90.0, cov=6.0)) == WITHHOLD
    assert m.state == "SPOOF_REJECTED" and m.mode == "SPOOF_REJECTED"
    assert m.events[-1][1] == "SPOOF_REJECTED"


def test_outliers_with_uncertain_filter_go_to_reacquire_not_spoof():
    m = IntegrityMonitor(CFG)
    m.on_fix(fix(0), pred(80.0, cov=40.0))
    m.on_fix(fix(1), pred(80.0, cov=40.0))
    assert m.state == "REACQUIRE"


def test_ramp_spoof_detected_by_cusum():
    m = IntegrityMonitor(CFG)
    decisions = [m.on_fix(fix(t), pred(6.0)) for t in range(6)]           # NIS 6 < 9.21 each, but persistently high
    assert m.state == "SPOOF_REJECTED"
    assert decisions[-1] == WITHHOLD and all(d == APPLY for d in decisions[:-1])
    assert [kind for _, kind, _ in m.events] == ["SPOOF_REJECTED"]        # h=15, +3 per fix -> 6th fix


def test_cusum_does_not_fire_on_healthy_nis():
    m = IntegrityMonitor(CFG)
    rng = np.random.default_rng(0)
    for t in range(2000):
        m.on_fix(fix(t), pred(float(rng.chisquare(2) * 0.85)))            # calibrated-ish, slightly conservative
        if m.state != "LOCK":
            pytest.fail(f"false alarm at {t}")


def test_poor_quality_fix_drops_to_dr_and_is_not_applied():
    m = IntegrityMonitor(CFG)
    assert m.on_fix(fix(0, sats=3), pred()) == WITHHOLD and m.state == "DR"
    m2 = IntegrityMonitor(CFG)
    assert m2.on_fix(fix(0, acc=45.0), pred()) == WITHHOLD and m2.state == "DR"


def test_no_fix_timeout_goes_to_dr():
    m = IntegrityMonitor(CFG)
    m.on_fix(fix(0), pred())
    m.on_seen(0.0)
    m.on_tick(0.9)
    assert m.state == "LOCK"
    m.on_tick(1.2)
    assert m.state == "DR" and m.mode == "DR"


def _to_dr(m):
    m.on_fix(fix(0), pred())
    m.on_seen(0.0)
    m.on_tick(5.0)
    assert m.state == "DR"


def test_reacquire_after_outage_needs_three_consistent_fixes_in_gate():
    m = IntegrityMonitor(CFG)
    _to_dr(m)
    d = straight_disp()
    out = [m.on_fix(fix(60 + k, e=600 + 10 * k), pred(2.0, cov=30.0, disp=d)) for k in range(3)]
    assert out == [WITHHOLD, WITHHOLD, APPLY]
    assert m.state == "LOCK" and m.reentered


def test_reacquire_rejects_course_and_speed_disagreement():
    m = IntegrityMonitor(CFG)
    _to_dr(m)
    for k in range(5):
        m.on_fix(fix(60 + k, psi=np.radians(40)), pred(2.0, disp=straight_disp()))    # 40 deg off the filter course
    assert m.state == "REACQUIRE"


def test_reacquire_rejects_mutually_inconsistent_fixes():
    m = IntegrityMonitor(CFG)
    _to_dr(m)
    for k in range(6):
        m.on_fix(fix(60 + k, e=600 + 200 * (k % 2)), pred(2.0, disp=straight_disp()))
    assert m.state == "REACQUIRE"


def test_deadlock_breaker_after_outage_accepts_consistent_fixes_outside_gate():
    """Filter is overconfident (NIS always above the gate) but the fixes are self-consistent and agree with speed/course."""
    m = IntegrityMonitor(CFG)
    _to_dr(m)
    d = straight_disp()
    res = [m.on_fix(fix(60 + k, e=600 + 10 * k), pred(50.0, cov=20.0, disp=d)) for k in range(10)]
    assert res[-1] == APPLY and m.state == "LOCK"
    assert all(r == WITHHOLD for r in res[:-1])


def test_rejected_spoof_never_gets_the_deadlock_shortcut():
    m = IntegrityMonitor(CFG)
    m.on_fix(fix(0), pred(80.0, cov=6.0))
    m.on_fix(fix(1), pred(80.0, cov=6.0))
    assert m.state == "SPOOF_REJECTED"
    d = straight_disp()
    res = [m.on_fix(fix(2 + k, e=500 + 10 * k), pred(80.0, cov=6.0, disp=d)) for k in range(30)]
    assert all(r == WITHHOLD for r in res) and m.state == "SPOOF_REJECTED"


def test_spoof_rejected_recovers_when_fixes_return_to_the_gate():
    m = IntegrityMonitor(CFG)
    m.on_fix(fix(0), pred(80.0, cov=6.0))
    m.on_fix(fix(1), pred(80.0, cov=6.0))
    d = straight_disp()
    out = [m.on_fix(fix(2 + k, e=10 * k), pred(2.0, cov=8.0, disp=d)) for k in range(3)]
    assert out[-1] == APPLY and m.state == "LOCK"


def test_borderline_double_outlier_is_not_labelled_a_spoof():
    """NIS ~13 twice in a row (e.g. a filter misfit in a manoeuvre) re-verifies via REACQUIRE, not SPOOF_REJECTED."""
    m = IntegrityMonitor(CFG)
    m.on_fix(fix(0), pred(13.0, cov=6.0))
    m.on_fix(fix(1), pred(14.0, cov=6.0))
    assert m.state == "REACQUIRE" and m.mode == "DR"
    assert not any(kind == "SPOOF_REJECTED" for _, kind, _ in m.events)


def test_deadlock_breaker_does_not_need_the_filter_course_to_be_right():
    """After a long outage the filter heading can be 60 deg off; genuine fixes still agree with their own velocity."""
    m = IntegrityMonitor(CFG)
    _to_dr(m)
    d = straight_disp()
    res = [m.on_fix(fix(60 + k, e=600 + 10 * k, psi=0.0), pred(80.0, cov=200.0, psi=np.radians(60), disp=d))
           for k in range(10)]
    assert res[-1] == APPLY and m.state == "LOCK" and m.reentered


def test_fixes_inconsistent_with_their_own_velocity_are_never_forced_in():
    """A position ramp that the reported velocity contradicts stays rejected."""
    m = IntegrityMonitor(CFG)
    _to_dr(m)
    d = straight_disp()
    res = [m.on_fix(fix(60 + k, e=600 + 40 * k, speed=10.0, psi=0.0), pred(80.0, cov=200.0, disp=d))
           for k in range(30)]
    assert all(r == WITHHOLD for r in res) and m.state == "REACQUIRE"


def test_confident_filter_borderline_misfit_never_gets_the_deadlock_shortcut():
    """A slowly diverging (ramp) source must not be let back in after 10 self-consistent fixes."""
    m = IntegrityMonitor(CFG)
    m.on_fix(fix(0), pred(13.0, cov=6.0))
    m.on_fix(fix(1), pred(14.0, cov=6.0))
    assert m.state == "REACQUIRE"
    d = straight_disp()
    res = [m.on_fix(fix(2 + k, e=10 * k), pred(15.0, cov=6.0, disp=d)) for k in range(30)]
    assert all(r == WITHHOLD for r in res)
    assert m.state == "SPOOF_REJECTED"                        # persistent suspect escalates after 5 s
    esc = [t for t, kind, _ in m.events if kind == "SPOOF_REJECTED"]
    assert len(esc) == 1 and 5.0 < esc[0] - 1 <= 9.0
