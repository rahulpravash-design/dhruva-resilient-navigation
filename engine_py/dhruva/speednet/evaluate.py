"""Held-out evaluation of SpeedNet in the engine (SYNTHETIC test drives, aligned pipeline, TFLite model).

usage: python -m dhruva.speednet.evaluate [--drives N]
Compares the P3 engine (NHC, no ML) with the same engine plus the SpeedNet pseudo-measurement, on the fixed `test`
split, over injected GNSS outages of 30 s and 60 s. Writes models/speednet_eval.json.
"""
import argparse
import json
from pathlib import Path

import numpy as np
from joblib import Parallel, delayed

from ..align import MountAligner
from ..config import load_config
from ..ekf import Engine, EngineFlags
from ..eval import run_fixture, score_outage
from ..metrics import summarize
from ..sim import apply_outage, simulate
from .data import random_scenario, split_seeds
from .infer import MODELS_DIR, TfliteSpeedEstimator

DURATION_S = 120.0


def outage_start(sim, seed, length):
    """Seeded start time (>= 35 s, so alignment has converged) where the vehicle is moving > 5 m/s."""
    rng = np.random.default_rng(seed + 7)
    t, v = sim.truth["t"], sim.truth["v"]
    ok = np.flatnonzero((t >= 35.0) & (t <= DURATION_S - length - 2.0) & (v > 5.0))
    return float(t[rng.choice(ok)]) if len(ok) else None


def run_case(seed, length, use_ml, model_dir):
    cfg = load_config()
    sim = simulate(random_scenario(seed, DURATION_S))
    t0 = outage_start(sim, seed, length)
    if t0 is None:
        return None
    df = apply_outage(sim.df, t0, length)
    est = TfliteSpeedEstimator(Path(model_dir) / "speednet.tflite") if use_ml else None
    tl, _ = run_fixture(df, MountAligner(cfg), Engine, EngineFlags(use_ml=use_ml), cfg, est)
    # The last row inside the outage is scored, before GNSS returns.
    return {"seed": seed, "length": length, "use_ml": use_ml, **score_outage(df, tl, t0, t0 + length - 0.02)}


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--drives", type=int, default=None)
    ap.add_argument("--model-dir", default=str(MODELS_DIR))
    ap.add_argument("--jobs", type=int, default=4)
    args = ap.parse_args(argv)
    seeds = split_seeds("test")[: args.drives]
    jobs = [(s, 30.0 if i % 2 == 0 else 60.0, ml) for i, s in enumerate(seeds) for ml in (False, True)]
    rows = [r for r in Parallel(n_jobs=args.jobs)(delayed(run_case)(s, L, ml, args.model_dir) for s, L, ml in jobs)
            if r]
    result = {"label": "SYNTHETIC", "split": "test", "drives": len(seeds), "pipeline": "MountAligner + EKF",
              "note": "Assumed vibration model; not evidence of real-world performance.", "by_outage_s": {}}
    for L in (30.0, 60.0):
        entry = {}
        for ml in (False, True):
            sel = [r for r in rows if r["length"] == L and r["use_ml"] == ml]
            entry["with_speednet" if ml else "without_speednet"] = {
                "n": len(sel),
                "endpoint_err_m": summarize([r["endpoint_err_m"] for r in sel]),
                "drift_pct": summarize([r["drift_pct"] for r in sel]),
                "coverage_pct": summarize([r["coverage_pct"] for r in sel]),
                "ps_pass_rate": float(np.mean([r["ps_pass"] for r in sel])) if sel else float("nan"),
            }
        result["by_outage_s"][str(int(L))] = entry
    Path(args.model_dir, "speednet_eval.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
