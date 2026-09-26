"""Evaluation harness: outage x ablation x attack matrix over seeded SYNTHETIC drives (master spec section 8).

Each case is one engine run on one trajectory. `run_eval` returns the case dicts; `write_cases` stores them as
`cases.jsonl` (one JSON object per line, sorted by id) so RESULTS.md can be regenerated and traced by SHA-256.
"""
import hashlib
import json
from pathlib import Path

import numpy as np
import yaml
from joblib import Parallel, delayed

from ..align import MountAligner
from ..config import DEFAULT_PATH, load_config
from ..ekf import BaselineEngine, Engine, EngineFlags
from ..geo import to_enu
from ..metrics import outage_metrics, reentry_jump_m, time_to_detect_s
from ..sim import apply_outage, apply_spoof_ramp, apply_spoof_step, simulate
from ..speednet.data import random_scenario, split_seeds
from ..speednet.infer import MODELS_DIR, TfliteSpeedEstimator
from .replay import replay

ABLATIONS = {          # cumulative ladder: each row adds one component to the previous one
    "baseline": (BaselineEngine, EngineFlags(use_nhc=False, use_zupt=False)),
    "physics": (Engine, EngineFlags(use_nhc=False)),
    "nhc": (Engine, EngineFlags()),
    "ml": (Engine, EngineFlags(use_ml=True)),
    "integrity": (Engine, EngineFlags(use_ml=True, use_integrity=True)),
}
_ESTIMATOR = {}


def _estimator():
    if "m" not in _ESTIMATOR:
        _ESTIMATOR["m"] = TfliteSpeedEstimator(MODELS_DIR / "speednet.tflite")
    return _ESTIMATOR["m"]


def _clean(o):
    if isinstance(o, dict):
        return {k: _clean(v) for k, v in o.items()}
    if isinstance(o, (list, tuple)):
        return [_clean(v) for v in o]
    if isinstance(o, (float, np.floating)):
        return None if not np.isfinite(o) else round(float(o), 6)
    if isinstance(o, (bool, np.bool_)):
        return bool(o)
    if isinstance(o, np.integer):
        return int(o)
    return o


def _run(df, ablation, cfg):
    cls, flags = ABLATIONS[ablation]
    eng = replay(df, MountAligner(cfg), cls, flags, cfg, _estimator() if flags.use_ml else None)
    tl = eng.timeline_df()
    idx = np.clip(np.searchsorted(df["t"].to_numpy(), tl["t"].to_numpy()), 0, len(df) - 1)
    tl["truth_lat"] = df["truth_lat"].to_numpy()[idx]
    tl["truth_lon"] = df["truth_lon"].to_numpy()[idx]
    return tl, eng.events_df()


def _false_alarms(events, t_end):
    """SPOOF_REJECTED transitions before t_end (the clean part of a run)."""
    return int(sum(1 for t, kind, _ in events.itertuples(index=False) if kind == "SPOOF_REJECTED" and t < t_end))


def _write_sample(out_dir, case, tl):
    d = Path(out_dir) / case["id"]
    d.mkdir(parents=True, exist_ok=True)
    rows = tl.to_dict("records")
    (d / "timeline.jsonl").write_text(
        "".join(json.dumps(_clean(r), sort_keys=True) + "\n" for r in rows), encoding="utf-8", newline="\n")
    (d / "metrics.json").write_text(json.dumps(_clean(case), indent=2, sort_keys=True) + "\n", encoding="utf-8",
                                    newline="\n")


def _outage_start(sim, seed, length, k, ev):
    d = ev["drive"]
    rng = np.random.default_rng([seed, int(length), k, ev["seed_offset"]])
    t, v = sim.truth["t"], sim.truth["v"]
    ok = np.flatnonzero((t >= d["outage_start_min_s"]) & (t <= d["duration_s"] - length - d["tail_s"]) & (v > 5.0))
    return float(t[rng.choice(ok)]) if len(ok) else None


def _task(seed, is_sample_traj, ev):
    """All cases of one trajectory: simulate once, then every outage window and attack for every ablation."""
    cfg = load_config()
    d, att, smp = ev["drive"], ev["attack"], ev["sample"]
    cases = []

    def record(case, tl, sampled):
        if sampled and smp.get("dir"):
            _write_sample(smp["dir"], case, tl)
        cases.append(case)

    sim = simulate(random_scenario(seed, d["duration_s"]))
    for length in ev["outage_s"]:
        for k in range(ev["windows_per_trajectory"]):
            t0 = _outage_start(sim, seed, length, k, ev)
            if t0 is None:
                continue
            t1 = t0 + length
            df = apply_outage(sim.df, t0, length)
            df = df[df["t"] <= t1 + d["tail_s"]].reset_index(drop=True)
            for ab in ev["ablations"]:
                tl, events = _run(df, ab, cfg)
                idx = np.clip(np.searchsorted(df["t"].to_numpy(), tl["t"].to_numpy()), 0, len(df) - 1)
                m = outage_metrics(tl["t"], tl["lat"], tl["lon"], tl["cov95_m"], df["truth_lat"].to_numpy()[idx],
                                   df["truth_lon"].to_numpy()[idx], t0, t1 - 0.02)
                m["reentry_jump_m"] = reentry_jump_m(tl["t"], tl["lat"], tl["lon"], tl["speed"], t1)
                m["time_to_detect_s"] = time_to_detect_s(tl["t"], (tl["mode"] != "GNSS").to_numpy(), t0)
                m["false_alarms"] = _false_alarms(events, df["t"].iloc[-1] + 1.0)
                m["clean_s"] = float(df["t"].iloc[-1] - df["t"].iloc[0])
                case = {"id": f"outage-s{seed}-L{length}-w{k}-{ab}", "kind": "outage", "seed": seed,
                        "ablation": ab, "outage_s": length, "t0": t0, "attack": "none", "metrics": m}
                record(case, tl, is_sample_traj and length == smp.get("outage_s") and k == 0
                       and ab in smp.get("ablations", ()))
    for attack in ("step", "ramp"):
        rng = np.random.default_rng([seed, 1 if attack == "step" else 2, ev["seed_offset"]])
        ts = float(rng.uniform(*att["start_range_s"]))
        base = sim.df[sim.df["t"] <= att["duration_s"]].reset_index(drop=True)
        df = apply_spoof_step(base, ts, att["step_m"]) if attack == "step" else apply_spoof_ramp(base, ts, att["ramp_mps"])
        for ab in ev["ablations"]:
            tl, events = _run(df, ab, cfg)
            idx = np.clip(np.searchsorted(df["t"].to_numpy(), tl["t"].to_numpy()), 0, len(df) - 1)
            e, n = to_enu(tl["lat"].to_numpy(), tl["lon"].to_numpy(), df["truth_lat"].to_numpy()[idx],
                          df["truth_lon"].to_numpy()[idx])
            err = np.hypot(e, n)
            after = tl["t"].to_numpy() >= ts
            ttd = time_to_detect_s(tl["t"], (tl["mode"] == "SPOOF_REJECTED").to_numpy(), ts)
            m = {"spoof_start_s": ts, "detected": bool(np.isfinite(ttd)), "time_to_detect_s": ttd,
                 "max_err_after_m": float(err[after].max()), "final_err_m": float(err[-1]),
                 "false_alarms": _false_alarms(events, ts), "clean_s": float(ts - df["t"].iloc[0])}
            case = {"id": f"attack-{attack}-s{seed}-{ab}", "kind": "attack", "seed": seed, "ablation": ab,
                    "outage_s": None, "t0": ts, "attack": attack, "metrics": m}
            record(case, tl, is_sample_traj and ab in smp.get("ablations", ()))
    return cases


def eval_config(path):
    path = Path(path)
    text = path.read_text(encoding="utf-8")
    ev = yaml.safe_load(text)
    ev["_sha256"] = hashlib.sha256(text.encode()).hexdigest()
    ev["_config"] = f"configs/{path.name}"
    return ev


def run_eval(ev, jobs=None, root=None):
    """Run the whole matrix. Returns cases sorted by id."""
    seeds = split_seeds(ev["split"])[: ev["trajectories"]]
    ev = {**ev, "sample": {**ev.get("sample", {}), "dir": _abs(ev.get("sample", {}).get("dir"), root)}}
    out = Parallel(n_jobs=jobs or ev.get("jobs", 4))(delayed(_task)(s, i < ev["sample"].get("trajectories", 0), ev)
                                                     for i, s in enumerate(seeds))
    return sorted((c for cs in out for c in cs), key=lambda c: c["id"])


def _abs(p, root):
    return None if not p else str(Path(root or ".") / p)


def write_cases(cases, out_dir, ev):
    """Write cases.jsonl + summary.json; returns the SHA-256 of cases.jsonl."""
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    body = "".join(json.dumps(_clean(c), sort_keys=True) + "\n" for c in cases)
    (out / "cases.jsonl").write_text(body, encoding="utf-8", newline="\n")
    digest = hashlib.sha256(body.encode()).hexdigest()
    engine_cfg = DEFAULT_PATH.read_text(encoding="utf-8")
    summary = {"label": ev["label"], "grade": ev["grade"], "split": ev["split"], "trajectories": ev["trajectories"],
               "cases": len(cases), "cases_sha256": digest, "eval_yaml_sha256": ev.get("_sha256"),
               "engine_json_sha256": hashlib.sha256(engine_cfg.encode()).hexdigest()}
    (out / "summary.json").write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8",
                                      newline="\n")
    return summary
