import numpy as np
import pytest

from dhruva.geo import to_enu
from dhruva.sim import (
    FIXTURE_SCENARIOS,
    Scenario,
    Segment,
    apply_outage,
    apply_spoof_ramp,
    apply_spoof_step,
    simulate,
)
from dhruva.sim.gnss import GnssParams
from dhruva.sim.imu import ImuParams

STRAIGHT_30 = Scenario("s", (Segment("straight", 30, 15.0),), seed=7)


def test_seeded_determinism():
    a, b = simulate(STRAIGHT_30), simulate(STRAIGHT_30)
    assert a.df.equals(b.df)
    other = simulate(Scenario("s", STRAIGHT_30.segments, seed=8))
    assert not a.df["ax"].equals(other.df["ax"])


def test_zero_noise_imu_reproduces_truth():
    sc = Scenario("t", (Segment("straight", 8, 14.0), Segment("turn", 6, 8.0, 90.0), Segment("straight", 8, 14.0)),
                  seed=3, imu=ImuParams.zero())
    r = simulate(sc)
    tr = r.truth
    R = np.array(r.meta["R_pv"])
    dt = 0.01
    f = r.df[["ax", "ay", "az"]].to_numpy() @ R          # phone -> vehicle
    w = r.df[["gx", "gy", "gz"]].to_numpy() @ R
    psi = tr["psi"][0]
    v = tr["v"][0] * np.array([np.cos(psi), np.sin(psi)])
    p = np.array([tr["e"][0], tr["n"][0]])
    for k in range(len(f) - 1):
        c, s = np.cos(psi), np.sin(psi)
        a = np.array([c * f[k, 0] - s * f[k, 1], s * f[k, 0] + c * f[k, 1]])
        p = p + v * dt + 0.5 * a * dt ** 2
        v = v + a * dt
        psi += w[k, 2] * dt
    assert np.hypot(p[0] - tr["e"][-1], p[1] - tr["n"][-1]) < 1.0
    assert abs(psi - tr["psi"][-1]) < 1e-3


def test_mount_is_a_rotation():
    R = np.array(simulate(STRAIGHT_30).meta["R_pv"])
    assert np.allclose(R @ R.T, np.eye(3), atol=1e-12) and np.linalg.det(R) == pytest.approx(1.0)


def test_left_turn_increases_psi():
    r = simulate(Scenario("l", (Segment("straight", 2, 10.0), Segment("turn", 6, 8.0, 90.0),
                                Segment("straight", 4, 10.0)), seed=1))
    assert np.degrees(r.truth["psi"][-1] - r.truth["psi"][0]) == pytest.approx(90.0, abs=5.0)


def test_gnss_rate_and_noise_level():
    sc = Scenario("long", (Segment("straight", 300, 15.0),), seed=5, gnss=GnssParams(pos_noise_std_m=3.5))
    df = simulate(sc).df
    fixes = df[df["gnss_new"]]
    assert len(fixes) == 300
    e, n = to_enu(fixes["gnss_lat"].to_numpy(), fixes["gnss_lon"].to_numpy(),
                  fixes["truth_lat"].to_numpy(), fixes["truth_lon"].to_numpy())
    assert 3.0 < np.std(np.concatenate([e, n])) < 4.0
    assert fixes["gnss_acc"].iloc[0] > 3.0
    assert df.loc[~df["gnss_new"], "gnss_lat"].isna().all()


def test_stop_scenario_actually_stops():
    r = simulate(FIXTURE_SCENARIOS["stop_go"])
    stopped = (r.truth["t"] > 14) & (r.truth["t"] < 19)
    assert stopped.any() and np.max(r.truth["v"][stopped]) < 0.05


def _shift_m(a, b, rows):
    e, n = to_enu(a.loc[rows, "gnss_lat"].to_numpy(), a.loc[rows, "gnss_lon"].to_numpy(),
                  b.loc[rows, "gnss_lat"].to_numpy(), b.loc[rows, "gnss_lon"].to_numpy())
    return np.hypot(e, n)


def test_outage_window_and_no_mutation():
    df = simulate(STRAIGHT_30).df
    before = df.copy()
    out = apply_outage(df, 10.0, 5.0)
    win = (out["t"] >= 10.0) & (out["t"] < 15.0)
    assert not out.loc[win, "gnss_valid"].any() and not out.loc[win, "gnss_new"].any()
    assert out.loc[win, ["gnss_lat", "gnss_lon"]].isna().all().all()
    assert out.loc[~win, "gnss_valid"].all()
    assert (out.loc[~win, "gnss_new"] == df.loc[~win, "gnss_new"]).all()
    assert df.equals(before)


def test_spoof_step_500m_after_start_only():
    df = simulate(STRAIGHT_30).df
    out = apply_spoof_step(df, 15.0, 500.0)
    late = df["gnss_new"] & (df["t"] >= 15.0)
    early = df["gnss_new"] & (df["t"] < 15.0)
    assert np.allclose(_shift_m(out, df, late), 500.0, atol=0.5)
    assert np.allclose(_shift_m(out, df, early), 0.0, atol=1e-6)


def test_spoof_ramp_grows_2mps():
    df = simulate(STRAIGHT_30).df
    out = apply_spoof_ramp(df, 10.0, 2.0)
    late = df["gnss_new"] & (df["t"] >= 10.0)
    expected = 2.0 * (df.loc[late, "t"].to_numpy() - 10.0)
    assert np.allclose(_shift_m(out, df, late), expected, atol=0.5)
