"""SpeedNet plumbing that does not need TensorFlow: features, window buffer, EKF wiring with fake estimators."""
import numpy as np
import pytest

from dhruva.ekf import Engine, EngineFlags
from dhruva.eval import load_fixture, run_fixture
from dhruva.speednet.features import N_FEATURES, WINDOW, WindowBuffer, feature_row, feature_rows


def test_feature_rows_layout():
    a = np.array([[1.0, 2.0, 10.0], [0.0, 0.0, 9.80665]])
    w = np.array([[0.1, 0.2, 0.3], [0.0, 0.0, 0.0]])
    f = feature_rows(a, w)
    assert f.shape == (2, N_FEATURES)
    assert np.allclose(f[0], [1.0, 2.0, 10.0 - 9.80665, 0.1, 0.2, 0.3, np.hypot(1.0, 2.0)])
    assert np.allclose(f[1], np.zeros(7))
    assert np.allclose(feature_row(a[0], w[0]), f[0])


def test_window_buffer():
    buf = WindowBuffer()
    for i in range(WINDOW - 1):
        buf.push(np.full(N_FEATURES, i))
    assert not buf.full
    buf.push(np.full(N_FEATURES, 99))
    assert buf.full and buf.window().shape == (WINDOW, N_FEATURES) and buf.window()[-1, 0] == 99
    buf.push(np.full(N_FEATURES, 100))
    assert buf.window()[0, 0] == 1                       # oldest sample dropped
    buf.clear()
    assert not buf.full


class ConstEstimator:
    def __init__(self, mu, logvar):
        self.mu, self.logvar, self.calls = mu, logvar, 0

    def estimate(self, window):
        assert window.shape == (WINDOW, N_FEATURES)
        self.calls += 1
        return self.mu, self.logvar


def test_use_ml_requires_estimator():
    df, _, al = load_fixture("tunnel_straight")
    with pytest.raises(ValueError, match="speed_estimator"):
        run_fixture(df, al, Engine, EngineFlags(use_ml=True))


def test_speednet_update_pulls_speed_and_is_logged():
    df, _, al = load_fixture("tunnel_straight")               # cruising at 16.7 m/s
    fake = ConstEstimator(mu=10.0, logvar=np.log(0.04))
    tl_off, log_off = run_fixture(df, al, Engine)
    tl_on, log_on = run_fixture(df, al, Engine, EngineFlags(use_ml=True), speed_estimator=fake)
    sn = log_on[log_on["kind"] == "speednet"]
    assert len(sn) > 100 and (log_off["kind"] != "speednet").all() and fake.calls == len(sn)
    assert sn["t"].min() >= 1.5                                # needs a full 1.5 s window first
    assert (sn["dof"] == 1).all()
    inside = (tl_on["t"] > 15) & (tl_on["t"] < 20)              # during the outage
    assert tl_on.loc[inside, "speed"].mean() < tl_off.loc[inside, "speed"].mean() - 3.0


def test_speednet_default_off_keeps_golden_behaviour():
    df, _, al = load_fixture("tunnel_turns")
    a, _ = run_fixture(df, al, Engine)
    b, _ = run_fixture(df, al, Engine, EngineFlags(use_ml=False))
    assert a.equals(b)
