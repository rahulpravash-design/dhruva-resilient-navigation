"""Unified fixture/replay CSV (100 Hz). Booleans are stored as 0/1; unknown values are blank.

Rows are at 100 Hz. On rows with gnss_new False every gnss_* / cn0 / sats value is blank.
gnss_valid is False while GNSS is unavailable (outage), True otherwise, including during spoofing.
"""
import pandas as pd

FIXTURE_COLUMNS = (
    "t", "ax", "ay", "az", "gx", "gy", "gz",
    "gnss_new", "gnss_lat", "gnss_lon", "gnss_acc", "gnss_speed", "gnss_bearing",
    "cn0_mean", "sats_used", "gnss_valid",
    "truth_lat", "truth_lon", "truth_speed", "speednet_mu", "speednet_logvar",
)
_BOOL = ("gnss_new", "gnss_valid")


def write_fixture(df, path):
    out = df[list(FIXTURE_COLUMNS)].copy()
    for c in _BOOL:
        out[c] = out[c].astype(int)
    out.to_csv(path, index=False, float_format="%.10g", lineterminator="\n")


def read_fixture(path):
    df = pd.read_csv(path)
    if tuple(df.columns) != FIXTURE_COLUMNS:
        raise ValueError(f"{path}: columns {list(df.columns)} != fixture columns")
    for c in _BOOL:
        df[c] = df[c].astype(bool)
    return df
