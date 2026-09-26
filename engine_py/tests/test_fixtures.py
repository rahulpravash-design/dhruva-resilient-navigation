import json
from pathlib import Path

import numpy as np
import pytest

from dhruva.io.fixture import FIXTURE_COLUMNS, read_fixture, write_fixture
from dhruva.sim import FIXTURE_SCENARIOS, simulate

FIX_DIR = Path(__file__).resolve().parents[2] / "data" / "fixtures"


@pytest.mark.parametrize("name", list(FIXTURE_SCENARIOS))
def test_fixture_is_valid_and_labelled(name):
    df = read_fixture(FIX_DIR / f"{name}.csv")
    meta = json.loads((FIX_DIR / f"{name}.meta.json").read_text())
    assert tuple(df.columns) == FIXTURE_COLUMNS
    assert len(df) <= 3000
    assert meta["label"] == "SYNTHETIC"
    assert np.all(np.diff(df["t"]) > 0)
    for t0, dur in meta["outages"]:
        win = (df["t"] >= t0) & (df["t"] < t0 + dur)
        assert win.sum() > 0
        assert not df.loc[win, "gnss_valid"].any() and not df.loc[win, "gnss_new"].any()
        assert df.loc[win, "gnss_lat"].isna().all()
    assert df["gnss_new"].sum() > 0


@pytest.mark.parametrize("name", list(FIXTURE_SCENARIOS))
def test_fixture_matches_scenario(name, tmp_path):
    p = tmp_path / "x.csv"
    write_fixture(simulate(FIXTURE_SCENARIOS[name]).df, p)
    a, b = read_fixture(p), read_fixture(FIX_DIR / f"{name}.csv")
    cols = [c for c in FIXTURE_COLUMNS if not c.startswith("speednet_")]   # those are filled by annotate_speednet.py
    assert np.allclose(a[cols].to_numpy(float), b[cols].to_numpy(float), rtol=1e-6, atol=1e-6, equal_nan=True)


@pytest.mark.parametrize("name", list(FIXTURE_SCENARIOS))
def test_fixture_speednet_columns_annotated(name):
    df = read_fixture(FIX_DIR / f"{name}.csv")
    have = df["speednet_mu"].notna()
    assert have.sum() > 100 and (have == df["speednet_logvar"].notna()).all()
    assert df.loc[have, "speednet_mu"].between(0, 40).all()
    assert df.loc[have, "speednet_logvar"].between(-6, 4).all()
    assert (df.loc[have, "t"] >= 1.5).all()                # first window needs 1.5 s


def test_read_rejects_wrong_columns(tmp_path):
    p = tmp_path / "bad.csv"
    p.write_text("t,ax\n0,1\n")
    with pytest.raises(ValueError, match="columns"):
        read_fixture(p)
