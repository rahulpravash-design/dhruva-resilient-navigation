import numpy as np
import pytest

from dhruva.legacy.baseline import run
from dhruva.legacy.contract import ContractError, validate_input, validate_output
from dhruva.legacy.synth import make_drive


@pytest.fixture
def trace():
    return make_drive(duration=30.0)[0]


def test_synth_trace_is_valid(trace):
    validate_input(trace)


def test_baseline_output_is_valid(trace):
    validate_output(run(trace), n_expected=len(trace["t"]))


def test_missing_field(trace):
    del trace["gz"]
    with pytest.raises(ContractError, match="missing"):
        validate_input(trace)


def test_length_mismatch(trace):
    trace["ax"] = trace["ax"][:-1]
    with pytest.raises(ContractError, match="length"):
        validate_input(trace)


def test_time_not_increasing(trace):
    trace["t"][5] = trace["t"][4]
    with pytest.raises(ContractError, match="increasing"):
        validate_input(trace)


def test_nan_imu(trace):
    trace["ax"][3] = np.nan
    with pytest.raises(ContractError, match="ax"):
        validate_input(trace)


def test_stale_position_on_invalid_row(trace):
    i = np.flatnonzero(~trace["gnss_valid"])[0]
    trace["gnss_lat"][i] = 28.0
    with pytest.raises(ContractError, match="NaN"):
        validate_input(trace)


def test_first_row_needs_fix(trace):
    trace["gnss_valid"][0] = False
    trace["gnss_lat"][0] = trace["gnss_lon"][0] = np.nan
    with pytest.raises(ContractError, match="first row"):
        validate_input(trace)


def test_bad_bearing(trace):
    trace["gnss_bearing"][0] = 360.0
    with pytest.raises(ContractError, match="bearing"):
        validate_input(trace)


@pytest.mark.parametrize("key,val,msg", [
    ("gnss_trust", 1.5, "gnss_trust"),
    ("heading", 360.0, "heading"),
    ("speed", -1.0, "speed"),
    ("cov_ee", -1.0, "cov_ee"),
    ("mode", "LOST", "mode"),
])
def test_bad_output(trace, key, val, msg):
    out = run(trace)
    out[key] = np.array(out[key], copy=True)
    out[key][2] = val
    with pytest.raises(ContractError, match=msg):
        validate_output(out)


def test_covariance_not_psd(trace):
    out = run(trace)
    out["cov_en"] = out["cov_en"].copy()
    out["cov_en"][2] = 10 * np.sqrt(out["cov_ee"][2] * out["cov_nn"][2])
    with pytest.raises(ContractError, match="positive"):
        validate_output(out)
