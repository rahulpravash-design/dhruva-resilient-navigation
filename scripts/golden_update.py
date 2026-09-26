"""Regenerate golden Python-engine outputs for the committed fixtures. Commit message must say why."""
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "engine_py"))

from dhruva.eval import load_fixture, run_fixture
from dhruva.sim import FIXTURE_SCENARIOS

OUT = ROOT / "engine_py" / "tests" / "golden"


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    for name in FIXTURE_SCENARIOS:
        df, _, aligner = load_fixture(name)
        timeline, log = run_fixture(df, aligner)
        timeline.to_csv(OUT / f"{name}.timeline.csv", index=False, float_format="%.10g", lineterminator="\n")
        log.to_csv(OUT / f"{name}.log.csv", index=False, float_format="%.10g", lineterminator="\n")
        print(f"{name}: {len(timeline)} ticks, {len(log)} updates")


if __name__ == "__main__":
    main()
