"""python -m dhruva.eval --config configs/eval.yaml [--out runs] [--results docs/RESULTS.md]"""
import argparse
from pathlib import Path

from .report import read_cases, render
from .runner import eval_config, run_eval, write_cases


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default="configs/eval.yaml")
    ap.add_argument("--out", default=None, help="runs directory (default: <repo>/runs)")
    ap.add_argument("--results", default=None, help="RESULTS.md path (default: <repo>/docs/RESULTS.md)")
    ap.add_argument("--jobs", type=int, default=None)
    ap.add_argument("--trajectories", type=int, default=None, help="override the config (quick runs)")
    ap.add_argument("--report-only", action="store_true", help="regenerate RESULTS.md from an existing cases.jsonl")
    args = ap.parse_args(argv)
    cfg_path = Path(args.config).resolve()
    root = cfg_path.parent.parent
    ev = eval_config(cfg_path)
    if args.trajectories:
        ev["trajectories"] = args.trajectories
    out = Path(args.out) if args.out else root / "runs"
    run_dir = out / ev["id"]
    if not args.report_only:
        cases = run_eval(ev, args.jobs, root)
        summary = write_cases(cases, run_dir, ev)
    else:
        import json
        cases = read_cases(run_dir / "cases.jsonl")
        summary = json.loads((run_dir / "summary.json").read_text(encoding="utf-8"))
    results = Path(args.results) if args.results else root / "docs" / "RESULTS.md"
    results.write_text(render(cases, summary, ev), encoding="utf-8", newline="\n")
    print(f"{len(cases)} cases -> {run_dir}; report -> {results}")


if __name__ == "__main__":
    main()
