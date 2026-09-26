"""Integrity monitor inside the engine (SYNTHETIC drives, true-mount aligner so alignment is not the variable)."""
import numpy as np
import pytest

from dhruva.align import OracleAligner
from dhruva.ekf import BaselineEngine, Engine, EngineFlags
from dhruva.eval import load_fixture, replay
from dhruva.geo import to_enu
from dhruva.metrics import reentry_jump_m
from dhruva.sim import Scenario, Segment, apply_outage, simulate

S = Segment
DRIVE = (S("straight", 6, 12), S("straight", 6, 18), S("turn", 6, 10, 90), S("straight", 6, 14),
         S("turn", 6, 10, -90), S("straight", 6, 8), S("straight", 6, 16))     # 42 s of varied dynamics


def _err(sim, tl):
    j = np.clip(np.rint(tl["t"].to_numpy() * 100).astype(int), 0, len(sim.df) - 1)
    e, n = to_enu(tl["lat"].to_numpy(), tl["lon"].to_numpy(),
                  sim.df["truth_lat"].to_numpy()[j], sim.df["truth_lon"].to_numpy()[j])
    return np.hypot(e, n)


def _run(sim, **flags):
    eng = replay(sim.df, OracleAligner(sim.meta["R_pv"]), Engine, EngineFlags(**flags))
    return eng.timeline_df(), eng.events_df()


@pytest.fixture(scope="module")
def sim():
    return simulate(Scenario("i", DRIVE, seed=4))


@pytest.mark.parametrize("name", ["tunnel_straight", "tunnel_turns", "stop_go"])
def test_no_false_alarms_on_clean_fixtures(name):
    df, _, al = load_fixture(name)
    eng = replay(df, al, Engine, EngineFlags(use_integrity=True))
    ev = eng.events_df()
    assert not (ev["kind"] == "SPOOF_REJECTED").any()
    tl = eng.timeline_df()
    assert tl["gnss_trust"].between(0.0, 1.0).all()
    assert set(tl["mode"]) <= {"GNSS", "DR", "DEGRADED"}


def test_integrity_off_leaves_baseline_and_engine_behaviour_alone(sim):
    a, _ = _run(sim)
    b, ev = _run(sim, use_integrity=False)
    assert a.equals(b) and ev.empty


def test_step_spoof_is_rejected_quickly_and_position_holds(sim):
    off, _ = _run(sim, spoof=("step", 20.0, 500.0))
    on, ev = _run(sim, spoof=("step", 20.0, 500.0), use_integrity=True)
    det = ev[ev["kind"] == "SPOOF_REJECTED"]
    assert len(det) >= 1 and det["t"].iloc[0] - 20.0 <= 3.0
    after = on["t"] > 20.0
    assert _err(sim, off)[after].max() > 400.0                # without the guard the position follows the spoof
    assert _err(sim, on)[after].max() < 50.0                  # with it, dead reckoning carries on
    assert (on.loc[after & (on["t"] > 23.0), "mode"] == "SPOOF_REJECTED").all()


def test_ramp_spoof_is_eventually_rejected(sim):
    off, _ = _run(sim, spoof=("ramp", 12.0, 2.0))
    on, ev = _run(sim, spoof=("ramp", 12.0, 2.0), use_integrity=True)
    assert (ev["kind"] == "SPOOF_REJECTED").any()
    assert _err(sim, on).max() < _err(sim, off).max()


def test_genuine_reentry_after_outage_is_accepted_without_a_display_jump():
    sc = Scenario("o", DRIVE + (S("straight", 20, 14), S("straight", 12, 14)), seed=6)
    sim = simulate(sc)
    df = apply_outage(sim.df, 14.0, 40.0)
    eng = replay(df, OracleAligner(sim.meta["R_pv"]), Engine, EngineFlags(use_integrity=True))
    tl, ev = eng.timeline_df(), eng.events_df()
    lock = ev[(ev["kind"] == "LOCK") & (ev["t"] > 54.0)]
    assert len(lock) == 1 and lock["t"].iloc[0] - 54.0 <= 10.0
    assert not (ev["kind"] == "SPOOF_REJECTED").any()
    assert tl["mode"].iloc[-1] == "GNSS"
    assert reentry_jump_m(tl["t"], tl["lat"], tl["lon"], tl["speed"], 54.0, window_s=12.0) <= 2.0
    assert _err(sim, tl)[tl["t"] > 66.0].max() < 10.0


def test_baseline_engine_ignores_the_integrity_flag(sim):
    eng = replay(sim.df, OracleAligner(sim.meta["R_pv"]), BaselineEngine, EngineFlags(use_integrity=True))
    assert eng.monitor is None and eng.events_df().empty
