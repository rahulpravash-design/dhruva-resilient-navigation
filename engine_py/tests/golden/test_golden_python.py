"""Python engine vs committed golden outputs. Regenerate with `make golden-update` (state the reason)."""
from pathlib import Path

import numpy as np
import pandas as pd
import pytest

from dhruva.eval import load_fixture, run_fixture
from dhruva.sim import FIXTURE_SCENARIOS

GOLDEN = Path(__file__).resolve().parent

pytestmark = pytest.mark.golden


def _same(a, b):
    assert list(a.columns) == list(b.columns)
    assert len(a) == len(b)
    for c in a.columns:
        if pd.api.types.is_bool_dtype(a[c]):
            assert (a[c] == b[c].astype(bool)).all(), c
        elif not pd.api.types.is_numeric_dtype(a[c]):
            assert (a[c].astype(str) == b[c].astype(str)).all(), c
        else:
            tol = 1e-7 if c in ("lat", "lon") else 0
            assert np.allclose(a[c].to_numpy(float), b[c].to_numpy(float), rtol=1e-5, atol=max(tol, 1e-8),
                               equal_nan=True), c


@pytest.mark.parametrize("name", list(FIXTURE_SCENARIOS))
def test_python_engine_matches_golden(name):
    df, _, aligner = load_fixture(name)
    timeline, log = run_fixture(df, aligner)
    _same(timeline, pd.read_csv(GOLDEN / f"{name}.timeline.csv"))
    _same(log, pd.read_csv(GOLDEN / f"{name}.log.csv"))
