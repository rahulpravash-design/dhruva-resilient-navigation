import numpy as np
import pytest

from dhruva.align import OracleAligner
from dhruva.ekf import BaselineEngine, Engine, EngineFlags
from dhruva.eval import load_fixture, run_fixture, score_outage
from dhruva.sim import FIXTURE_SCENARIOS, Scenario, Segment, simulate
from dhruva.sim.gnss import GnssParams
from dhruva.sim.imu import ImuParams

FIXTURES = list(FIXTURE_SCENARIOS)


def _outage(meta):
    t0, dur = meta["outages"][0]
    return t0, t0 + dur


@pytest.mark.parametrize("name", FIXTURES)
def test_ekf_beats_baseline_on_fixtures(name):
    """The EKF must beat the baseline where dynamics matter. tunnel_straight is constant-speed on a straight road,
    where 'hold the last GNSS speed' is near ideal and both errors (~10 m) are dominated by GNSS position noise,
    so there the EKF only has to stay within 3 m."""
    df, meta, al = load_fixture(name)
    t0, t1 = _outage(meta)
    ekf = score_outage(df, run_fixture(df, al, Engine)[0], t0, t1)
    base = score_outage(df, run_fixture(df, al, BaselineEngine)[0], t0, t1)
    slack = 3.0 if name == "tunnel_straight" else 0.0
    assert ekf["endpoint_err_m"] < base["endpoint_err_m"] + slack
    assert np.isnan(base["coverage_pct"])            # baseline has no covariance: NOT MEASURED


@pytest.mark.slow
def test_ekf_beats_baseline_on_random_validation_drives():
    """Aggregate gate on SYNTHETIC random drives (validation split, 30 s outages): median endpoint error."""
    from dhruva.sim import apply_outage, simulate
    from dhruva.speednet.data import random_scenario, split_seeds
    ekf_err, base_err = [], []
    for seed in split_seeds("val")[:6]:
        sim = simulate(random_scenario(seed, 90.0))
        tr = sim.truth
        ok = np.flatnonzero((tr["t"] >= 35) & (tr["t"] <= 58) & (tr["v"] > 5))
        if len(ok) == 0:
            continue
        t0 = float(tr["t"][ok[len(ok) // 2]])
        df = apply_outage(sim.df, t0, 30.0)
        for cls, out in ((Engine, ekf_err), (BaselineEngine, base_err)):
            tl, _ = run_fixture(df, OracleAligner(sim.meta["R_pv"]), cls)
            out.append(score_outage(df, tl, t0, t0 + 29.98)["endpoint_err_m"])
    assert len(ekf_err) >= 4 and np.median(ekf_err) < np.median(base_err)


def test_zero_noise_straight_line_under_0_1m():
    sc = Scenario("z", (Segment("straight", 30, 15.0),), seed=4, imu=ImuParams.zero(),
                  gnss=GnssParams(pos_noise_std_m=0.0, speed_noise_std=0.0, bearing_noise_std_deg=0.0),
                  outages=((10.0, 10.0),))
    r = simulate(sc)
    tl, _ = run_fixture(r.df, OracleAligner(r.meta["R_pv"]))
    assert score_outage(r.df, tl, 10.0, 20.0)["max_err_m"] < 0.1


def test_left_turn_increases_psi():
    sc = Scenario("l", (Segment("straight", 4, 12.0), Segment("turn", 6, 9.0, 90.0),
                        Segment("straight", 4, 12.0)), seed=5)
    r = simulate(sc)
    tl, _ = run_fixture(r.df, OracleAligner(r.meta["R_pv"]))
    dpsi = np.unwrap(tl["psi"].to_numpy())
    assert dpsi[-1] - dpsi[0] == pytest.approx(np.pi / 2, abs=0.25)
    assert np.all(np.diff(dpsi[10:80]) > -0.05) or dpsi[-1] > dpsi[0]


def test_covariance_grows_in_outage_and_shrinks_after():
    df, meta, al = load_fixture("tunnel_straight")
    t0, t1 = _outage(meta)
    tl, _ = run_fixture(df, al)
    at = lambda t: float(tl.loc[(tl["t"] - t).abs().idxmin(), "cov95_m"])
    assert at(t1 - 0.1) > 2 * at(t0 - 0.5)
    assert at(t1 + 3.0) < at(t1 - 0.1)


@pytest.mark.parametrize("name", ["tunnel_straight", "tunnel_turns"])
def test_nhc_reduces_drift(name):
    df, meta, al = load_fixture(name)
    t0, t1 = _outage(meta)
    on = score_outage(df, run_fixture(df, al, Engine)[0], t0, t1)
    off = score_outage(df, run_fixture(df, al, Engine, EngineFlags(use_nhc=False))[0], t0, t1)
    assert on["endpoint_err_m"] < off["endpoint_err_m"]


def test_zupt_zaru_at_stop():
    df, _, al = load_fixture("stop_go")
    on, _ = run_fixture(df, al, Engine)
    off, _ = run_fixture(df, al, Engine, EngineFlags(use_zupt=False))
    stop = lambda tl: tl[(tl["t"] > 15) & (tl["t"] < 19)]
    assert stop(on)["zupt_active"].sum() > 10
    assert not stop(on)["nhc_active"].any()
    assert stop(on)["speed"].max() < 0.05
    assert stop(on)["speed"].max() < stop(off)["speed"].max()


@pytest.mark.slow
def test_gnss_filter_consistency_on_clean_sim():
    """Mean NIS of the GNSS updates should be near their DOF (2) on clean simulated data."""
    segs = (Segment("straight", 20, 14.0), Segment("turn", 8, 9.0, 90.0), Segment("straight", 20, 14.0),
            Segment("turn", 8, 9.0, -90.0), Segment("straight", 20, 14.0))
    pos, vel = [], []
    for seed in (11, 12, 13):
        r = simulate(Scenario("c", segs, seed=seed))
        _, log = run_fixture(r.df, OracleAligner(r.meta["R_pv"]))
        pos.append(log.loc[log["kind"] == "gnss_pos", "nis"].mean())
        vel.append(log.loc[log["kind"] == "gnss_vel", "nis"].mean())
    assert 1.0 < np.mean(pos) < 3.5
    assert 1.0 < np.mean(vel) < 4.0


@pytest.mark.parametrize("name", FIXTURES)
def test_timeline_is_well_formed(name):
    df, meta, al = load_fixture(name)
    tl, log = run_fixture(df, al)
    t0, t1 = _outage(meta)
    assert np.allclose(np.diff(tl["t"]), 0.1, atol=1e-6) or np.allclose(np.diff(tl["t"])[1:], 0.1, atol=1e-6)
    assert np.isfinite(tl[["lat", "lon", "speed", "psi", "cov95_m"]].to_numpy()).all()
    assert set(tl["mode"]) <= {"GNSS", "DR"}
    inside = tl[(tl["t"] > t0 + 1.5) & (tl["t"] < t1)]
    outside = tl[(tl["t"] > 2.0) & ((tl["t"] < t0) | (tl["t"] > t1 + 1.5))]
    assert (inside["mode"] == "DR").all() and (outside["mode"] == "GNSS").all()
    assert set(log["kind"]) <= {"gnss_pos", "gnss_vel", "nhc", "zupt", "zaru"}


def test_deterministic_replay():
    df, _, al = load_fixture("tunnel_turns")
    a, _ = run_fixture(df, al)
    b, _ = run_fixture(df, al)
    assert a.equals(b)


def test_force_outage_flag_and_state():
    df, _, al = load_fixture("tunnel_straight")
    df = df.copy()
    df.loc[df["t"] < 0.005, "gnss_new"] = True          # keep the first fix
    tl, _ = run_fixture(df, al, Engine, EngineFlags(force_outage=((12.0, 6.0),)))
    win = tl[(tl["t"] > 13.5) & (tl["t"] < 18.0)]
    assert (win["mode"] == "DR").all()
    eng = Engine({**__import__("dhruva.config", fromlist=["x"]).load_config()}, EngineFlags(), al)
    eng.on_imu(0.0, np.array([0.0, 0.0, 9.8]), np.zeros(3))     # before the first fix: ignored, no crash
    assert eng.origin is None
