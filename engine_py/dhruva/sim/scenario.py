"""Scenario -> SYNTHETIC fixture DataFrame (+ sim-only ground truth). Fully seeded."""
from dataclasses import dataclass, field

import numpy as np
import pandas as pd

from ..geo import to_latlon
from ..io.fixture import FIXTURE_COLUMNS
from .faults import apply_outage
from .gnss import GnssParams, gnss_columns
from .imu import ImuParams, mount_after_move, phone_imu, random_mount
from .trajectory import DT, Segment, make_trajectory


@dataclass(frozen=True)
class Scenario:
    name: str
    segments: tuple
    seed: int
    origin: tuple = (28.6139, 77.2090)
    outages: tuple = ()  # ((t_start, dur_s), ...) natural tunnel windows
    imu: ImuParams = field(default_factory=ImuParams)
    gnss: GnssParams = field(default_factory=GnssParams)
    max_tilt_deg: float = 60.0
    psi0: float = 0.3
    mount_move: tuple = None  # (t_start, dur_s, angle_deg, axis_in_vehicle_frame): phone moved mid-drive


@dataclass
class SimResult:
    df: pd.DataFrame  # fixture columns
    truth: dict       # sim-only ground truth: t, e, n, v, psi, omega, a_long
    meta: dict        # sim-only: R_pv, scenario name, seed, label


def simulate(sc):
    rng = np.random.default_rng(sc.seed)
    traj = make_trajectory(list(sc.segments), psi0=sc.psi0)
    R_pv = random_mount(rng, sc.max_tilt_deg)
    acc, gyr = phone_imu(traj, R_pv, sc.imu, rng, DT, sc.mount_move)
    gn = gnss_columns(traj, sc.origin, sc.gnss, rng)
    tlat, tlon = to_latlon(traj["e"], traj["n"], *sc.origin)
    nan = np.full(len(traj["t"]), np.nan)

    cols = {"t": traj["t"], "ax": acc[:, 0], "ay": acc[:, 1], "az": acc[:, 2],
            "gx": gyr[:, 0], "gy": gyr[:, 1], "gz": gyr[:, 2], **gn,
            "truth_lat": tlat, "truth_lon": tlon, "truth_speed": traj["v"],
            "speednet_mu": nan, "speednet_logvar": nan}
    df = pd.DataFrame({c: cols[c] for c in FIXTURE_COLUMNS})
    for t0, dur in sc.outages:
        df = apply_outage(df, t0, dur)
    R_final = R_pv if sc.mount_move is None else mount_after_move(R_pv, sc.mount_move)
    meta = {"R_pv": R_final.tolist(), "R_pv_initial": R_pv.tolist(), "scenario": sc.name, "seed": sc.seed,
            "label": "SYNTHETIC"}
    return SimResult(df, traj, meta)


def _seg(kind, dur, speed=0.0, turn_deg=0.0):
    return Segment(kind, dur, speed, turn_deg)


# Each 30 s clip starts with GNSS-available acceleration / turning so mount alignment can converge
# before the tunnel, as it would earlier in a real drive.
FIXTURE_SCENARIOS = {
    "tunnel_straight": Scenario(
        "tunnel_straight", (_seg("straight", 4, 10.0), _seg("straight", 26, 16.7)),
        seed=1, outages=((10.0, 16.0),)),
    "tunnel_turns": Scenario(
        "tunnel_turns",
        (_seg("straight", 3, 10.0), _seg("straight", 3, 14.0), _seg("turn", 6, 14.0, 90.0),
         _seg("straight", 4, 14.0), _seg("turn", 6, 14.0, -60.0), _seg("straight", 8, 14.0)),
        seed=2, outages=((12.0, 16.0),)),
    "stop_go": Scenario(
        "stop_go", (_seg("straight", 6, 13.9), _seg("stop", 14), _seg("straight", 10, 10.0)),
        seed=3, outages=((13.0, 10.0),)),
}
