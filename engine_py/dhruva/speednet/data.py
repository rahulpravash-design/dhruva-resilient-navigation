"""SYNTHETIC SpeedNet training data: random drives -> levelled 1.5 s windows labelled with true speed.

Windows are built from the *true* mount plus a small random alignment error (tilt ~1 deg, yaw ~2 deg), which stands
in for the imperfection of the real aligner. Vibration follows the ASSUMED model in sim/imu.py.
"""
from pathlib import Path

import numpy as np
import yaml

from ..sim import Scenario, Segment, simulate
from ..sim.imu import ImuParams, _rx, _ry, _rz
from .features import N_FEATURES, WINDOW, G, feature_rows

SPLITS_PATH = Path(__file__).resolve().parents[3] / "data" / "splits.yaml"
STRIDE = 10  # 10 Hz windows


def split_seeds(name, path=SPLITS_PATH):
    s = yaml.safe_load(Path(path).read_text())["synthetic"][name]
    return list(range(s["seed_start"], s["seed_start"] + s["count"]))


def random_scenario(seed, duration_s=60.0):
    """A random drive: cruising, accelerating/braking, turns and stops, with random vibration strength."""
    rng = np.random.default_rng(seed)
    segs, total, speed = [], 0.0, rng.uniform(6, 16)
    while total < duration_s:
        r = rng.random()
        dur = float(rng.uniform(3, 8))
        if r < 0.15:
            segs.append(Segment("stop", dur))
            speed = rng.uniform(6, 14)
        elif r < 0.45:
            segs.append(Segment("turn", dur, float(min(speed, 14.0)), float(rng.choice([-1, 1]) * rng.uniform(30, 110))))
        else:
            speed = float(np.clip(speed + rng.uniform(-6, 8), 3.0, 25.0))
            segs.append(Segment("straight", dur, speed))
        total += dur
    imu = ImuParams(vib_gain=float(rng.uniform(0.02, 0.05)))
    return Scenario(f"rand{seed}", tuple(segs), seed=seed, imu=imu)


def _alignment_error(rng):
    tilt_x, tilt_y = np.radians(rng.normal(0, 1.0, 2))
    yaw = np.radians(rng.normal(0, 2.0))
    return _rz(yaw) @ _rx(tilt_x) @ _ry(tilt_y)


def drive_windows(seed, duration_s=60.0):
    """(windows (m, 150, 7) float32, speeds (m,), t_end (m,)) for one synthetic drive."""
    sc = random_scenario(seed, duration_s)
    r = simulate(sc)
    rng = np.random.default_rng(seed + 10_000)
    R_vp_est = _alignment_error(rng) @ np.array(r.meta["R_pv"]).T
    df = r.df
    a_v = df[["ax", "ay", "az"]].to_numpy() @ R_vp_est.T
    w_v = df[["gx", "gy", "gz"]].to_numpy() @ R_vp_est.T
    feats = feature_rows(a_v, w_v, G).astype(np.float32)
    ends = np.arange(WINDOW - 1, len(df), STRIDE)
    wins = np.stack([feats[e - WINDOW + 1: e + 1] for e in ends])
    return wins, r.truth["v"][ends].astype(np.float32), r.truth["t"][ends]


def make_dataset(seeds, duration_s=60.0):
    parts = [drive_windows(s, duration_s) for s in seeds]
    X = np.concatenate([p[0] for p in parts])
    y = np.concatenate([p[1] for p in parts])
    assert X.shape[1:] == (WINDOW, N_FEATURES)
    return X, y
