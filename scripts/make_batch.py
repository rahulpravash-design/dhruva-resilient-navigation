"""Turn an evaluation run (runs/<id>/cases.jsonl + summary.json) into web/data/batch.js for the dashboard ablation table.

Numbers are copied from the run, never invented; the run's SHA-256 values are carried along for provenance.

    python scripts/make_batch.py [run_id]        (default: eval_quick)
"""
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "engine_py"))

from dhruva.eval.report import NAMES, ORDER, read_cases
from dhruva.eval.runner import _clean
from dhruva.metrics import summarize

OUT = ROOT / "web" / "data" / "batch.js"
OUTAGE_KEYS = ("endpoint_err_m", "drift_pct", "err_per_km_m", "reentry_jump_m")
ATTACK_KEYS = ("max_err_after_m", "final_err_m")


def _vals(cases, key):
    return [np.nan if c["metrics"].get(key) is None else c["metrics"][key] for c in cases]


def _outage_row(sel):
    row = {"n": len(sel), "ps_pass": int(sum(bool(c["metrics"]["ps_pass"]) for c in sel)),
           "coverage_median": summarize(_vals(sel, "coverage_pct"))["median"]}
    for k in OUTAGE_KEYS:
        row[k] = summarize(_vals(sel, k))
    return row


def _attack_row(sel, ablation):
    row = {"n": len(sel)}
    for k in ATTACK_KEYS:
        row[k] = summarize(_vals(sel, k))
    if ablation == "integrity":
        row["detected"] = int(sum(bool(c["metrics"]["detected"]) for c in sel))
        row["time_to_detect_s"] = summarize(_vals(sel, "time_to_detect_s"))
    return row


def main(run_id="eval_quick"):
    run = ROOT / "runs" / run_id
    cases = read_cases(run / "cases.jsonl")
    summary = json.loads((run / "summary.json").read_text(encoding="utf-8"))
    present = [a for a in ORDER if any(c["ablation"] == a for c in cases)]
    lengths = sorted({c["outage_s"] for c in cases if c["outage_s"] is not None})
    outage = {str(length): {a: _outage_row([c for c in cases if c["ablation"] == a and c["outage_s"] == length])
                            for a in present} for length in lengths}
    attack = {kind: {a: _attack_row([c for c in cases if c["ablation"] == a and c["attack"] == kind], a)
                     for a in present} for kind in ("step", "ramp")}
    integ = [c for c in cases if c["ablation"] == "integrity"]
    batch = {"label": summary["label"], "grade": summary["grade"], "split": summary["split"],
             "drives": summary["trajectories"], "cases": summary["cases"], "run_id": run_id,
             "cases_sha256": summary["cases_sha256"], "eval_yaml_sha256": summary["eval_yaml_sha256"],
             "engine_json_sha256": summary["engine_json_sha256"], "ablations": present,
             "names": {a: NAMES[a] for a in present}, "outage": outage, "attack": attack,
             "false_alarms": {"count": int(sum(c["metrics"]["false_alarms"] for c in integ)),
                              "clean_hours": sum(c["metrics"]["clean_s"] for c in integ) / 3600.0}}
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text("window.DHRUVA_BATCH = " + json.dumps(_clean(batch), separators=(",", ":")) + ";\n",
                   encoding="utf-8", newline="\n")
    print(f"wrote {OUT.relative_to(ROOT)}: {summary['trajectories']} drives, {summary['cases']} cases")


if __name__ == "__main__":
    main(*sys.argv[1:2])
