"""Fill speednet_mu / speednet_logvar in the committed fixtures (for Kotlin parity without TFLite on the JVM).

Values are what the Python aligned pipeline (MountAligner + TFLite model) feeds the EKF at each 10 Hz tick where a
window was available; all other rows stay blank. Re-run after retraining the model.
"""
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "engine_py"))

from dhruva.align import MountAligner
from dhruva.config import load_config
from dhruva.ekf import Engine, EngineFlags
from dhruva.eval import load_fixture, run_fixture
from dhruva.io.fixture import write_fixture
from dhruva.sim import FIXTURE_SCENARIOS
from dhruva.speednet.infer import TfliteSpeedEstimator


class Recording:
    def __init__(self, inner):
        self.inner, self.out = inner, []

    def estimate(self, window):
        r = self.inner.estimate(window)
        self.out.append(r)
        return r


def main():
    cfg = load_config()
    for name in FIXTURE_SCENARIOS:
        df, _, _ = load_fixture(name)
        rec = Recording(TfliteSpeedEstimator())
        _, log = run_fixture(df, MountAligner(cfg), Engine, EngineFlags(use_ml=True), cfg, rec)
        times = log.loc[log["kind"] == "speednet", "t"].to_numpy()
        assert len(times) == len(rec.out)
        idx = np.searchsorted(df["t"].to_numpy(), times - 1e-9)
        df["speednet_mu"] = np.nan
        df["speednet_logvar"] = np.nan
        df.loc[df.index[idx], "speednet_mu"] = [r[0] for r in rec.out]
        df.loc[df.index[idx], "speednet_logvar"] = [r[1] for r in rec.out]
        write_fixture(df, ROOT / "data" / "fixtures" / f"{name}.csv")
        print(f"{name}: annotated {len(idx)} ticks")


if __name__ == "__main__":
    main()
