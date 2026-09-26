"""Regenerate the 3 committed SYNTHETIC fixtures from their scenarios (deterministic)."""
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "engine_py"))

from dhruva.io.fixture import write_fixture
from dhruva.sim import FIXTURE_SCENARIOS, simulate

OUT = ROOT / "data" / "fixtures"


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    for name, sc in FIXTURE_SCENARIOS.items():
        res = simulate(sc)
        write_fixture(res.df, OUT / f"{name}.csv")
        meta = {**res.meta, "outages": [list(o) for o in sc.outages], "rows": len(res.df)}
        (OUT / f"{name}.meta.json").write_text(json.dumps(meta, indent=2) + "\n")
        print(f"{name}: {len(res.df)} rows")


if __name__ == "__main__":
    main()
