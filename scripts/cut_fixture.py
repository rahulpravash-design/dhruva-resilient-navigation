"""Cut a window from a unified 100 Hz fixture CSV (e.g. around a real tunnel) into a small fixture.

usage: python scripts/cut_fixture.py IN.csv OUT.csv T_START DURATION_S
"""
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "engine_py"))

from dhruva.io.fixture import read_fixture, write_fixture

MAX_ROWS = 3000


def main(argv):
    src, dst, t0, dur = argv[1], argv[2], float(argv[3]), float(argv[4])
    df = read_fixture(src)
    win = df[(df["t"] >= t0) & (df["t"] < t0 + dur)].copy()
    if len(win) > MAX_ROWS:
        raise SystemExit(f"window has {len(win)} rows; fixtures are limited to {MAX_ROWS}")
    if len(win) == 0:
        raise SystemExit("window is empty")
    win["t"] = win["t"] - win["t"].iloc[0]
    write_fixture(win, dst)
    print(f"wrote {len(win)} rows to {dst}")


if __name__ == "__main__":
    main(sys.argv)
