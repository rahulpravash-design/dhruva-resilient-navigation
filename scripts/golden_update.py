"""Regenerate golden Python-engine outputs for the committed fixtures. Commit message must say why."""
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "engine_py"))

from dhruva.align import MountAligner
from dhruva.config import load_config
from dhruva.eval import load_fixture, run_fixture
from dhruva.sim import FIXTURE_SCENARIOS

OUT = ROOT / "engine_py" / "tests" / "golden"


def _write(df, path):
    df.to_csv(path, index=False, float_format="%.10g", lineterminator="\n")


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    for name in FIXTURE_SCENARIOS:
        df, _, oracle = load_fixture(name)
        for tag, aligner in (("", oracle), (".aligned", MountAligner(load_config()))):
            timeline, log = run_fixture(df, aligner)
            _write(timeline, OUT / f"{name}{tag}.timeline.csv")
            _write(log, OUT / f"{name}{tag}.log.csv")
            print(f"{name}{tag}: {len(timeline)} ticks, {len(log)} updates")


if __name__ == "__main__":
    main()
