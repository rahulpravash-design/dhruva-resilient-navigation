"""RESULTS.md rendering: deterministic, labelled SYNTHETIC, unmeasured sections say NOT MEASURED."""
from dhruva.eval.report import NM, render

EV = {"id": "eval", "_config": "configs/eval.yaml", "windows_per_trajectory": 1, "attack": {"start_range_s": [40.0, 70.0], "step_m": 500.0, "ramp_mps": 2.0}}
SUMMARY = {"label": "SYNTHETIC", "grade": "PHONE", "split": "test", "trajectories": 1, "cases": 3,
           "cases_sha256": "a" * 64, "eval_yaml_sha256": "b" * 64, "engine_json_sha256": "c" * 64}


def _case(ablation, kind, **m):
    return {"id": f"{kind}-{ablation}", "kind": kind, "seed": 1, "ablation": ablation,
            "outage_s": 30 if kind == "outage" else None, "t0": 50.0, "attack": "step" if kind == "attack" else "none",
            "metrics": m}


CASES = [
    _case("baseline", "outage", endpoint_err_m=90.0, drift_pct=20.0, err_per_km_m=200.0, coverage_pct=None,
          ps_pass=False, reentry_jump_m=80.0, time_to_detect_s=0.5, false_alarms=0, clean_s=60.0),
    _case("integrity", "outage", endpoint_err_m=3.0, drift_pct=0.7, err_per_km_m=7.0, coverage_pct=100.0,
          ps_pass=True, reentry_jump_m=0.1, time_to_detect_s=0.5, false_alarms=0, clean_s=60.0),
    _case("integrity", "attack", detected=True, time_to_detect_s=1.5, max_err_after_m=4.0, final_err_m=3.0,
          false_alarms=0, clean_s=50.0),
]


def test_render_is_deterministic_and_labelled():
    text = render(CASES, SUMMARY, EV)
    assert text == render(CASES, SUMMARY, EV)
    assert "All numbers on this page are SYNTHETIC" in text
    assert text.count(NM) >= 4  # map matching, vehicle-grade, real datasets, and the map-matching table row
    assert "1/1 (100 %)" in text and "0/1 (0 %)" in text
