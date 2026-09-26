"""Batch replay of a fixture through an engine, and per-outage scoring."""
import json
from pathlib import Path

import numpy as np

from ..align import OracleAligner
from ..config import load_config
from ..ekf import Engine, EngineFlags
from ..io.fixture import read_fixture
from ..metrics import outage_metrics

FIXTURE_DIR = Path(__file__).resolve().parents[3] / "data" / "fixtures"


def load_fixture(name, fixture_dir=None):
    """Return (df, meta, oracle_aligner) for a committed SYNTHETIC fixture."""
    d = Path(fixture_dir or FIXTURE_DIR)
    df = read_fixture(d / f"{name}.csv")
    meta = json.loads((d / f"{name}.meta.json").read_text())
    return df, meta, OracleAligner(meta["R_pv"])


def run_fixture(df, aligner, engine_cls=Engine, flags=None, config=None):
    """Feed every row to the engine. Returns (timeline_df, log_df)."""
    eng = engine_cls(config or load_config(), flags or EngineFlags(), aligner)
    t = df["t"].to_numpy()
    acc = df[["ax", "ay", "az"]].to_numpy()
    gyr = df[["gx", "gy", "gz"]].to_numpy()
    new = df["gnss_new"].to_numpy()
    fix_cols = {k: df[c].to_numpy() for k, c in (("lat", "gnss_lat"), ("lon", "gnss_lon"), ("acc", "gnss_acc"),
                                                  ("speed", "gnss_speed"), ("bearing", "gnss_bearing"))}
    for i in range(len(df)):
        eng.on_imu(t[i], acc[i], gyr[i])
        if new[i]:
            eng.on_gnss(t[i], {k: float(v[i]) for k, v in fix_cols.items()})
    return eng.timeline_df(), eng.log_df()


def score_outage(df, timeline, t0, t1):
    """Outage metrics of a timeline against the fixture's truth columns."""
    idx = np.clip(np.searchsorted(df["t"].to_numpy(), timeline["t"].to_numpy()), 0, len(df) - 1)
    return outage_metrics(timeline["t"], timeline["lat"], timeline["lon"], timeline["cov95_m"],
                          df["truth_lat"].to_numpy()[idx], df["truth_lon"].to_numpy()[idx], t0, t1)
