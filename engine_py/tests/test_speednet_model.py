"""SpeedNet model tests. Need TensorFlow (make setup-ml) and are slow (TF import), so they run in test-full."""
import json
import time

import numpy as np
import pytest

pytest.importorskip("tensorflow")
pytestmark = pytest.mark.slow

from dhruva.speednet.data import make_dataset
from dhruva.speednet.features import N_FEATURES, WINDOW
from dhruva.speednet.infer import MODELS_DIR, KerasSpeedEstimator, TfliteSpeedEstimator
from dhruva.speednet.model import build_model, gaussian_nll, trainable_params


def test_shape_range_and_param_budget():
    model = build_model()
    assert trainable_params(model) <= 50_000
    out = model.predict(np.random.default_rng(0).normal(size=(4, WINDOW, N_FEATURES)).astype("float32") * 50, verbose=0)
    assert out.shape == (4, 2)
    assert (out[:, 0] >= 0).all() and ((out[:, 1] >= -6.0) & (out[:, 1] <= 4.0)).all()


def test_nll_decreases_on_tiny_overfit_set():
    import keras
    X, y = make_dataset(range(9000, 9002), duration_s=20.0)
    X, y = X[:96], y[:96]
    mean, std = X.reshape(-1, N_FEATURES).mean(0), X.reshape(-1, N_FEATURES).std(0) + 1e-6
    model = build_model(mean, std)
    model.compile(optimizer=keras.optimizers.Adam(3e-3), loss=gaussian_nll)
    hist = model.fit(X, y, epochs=40, batch_size=32, verbose=0).history["loss"]
    assert hist[-1] < hist[0] - 1.0


def test_shipped_model_keras_vs_tflite_parity_and_latency():
    X, _ = make_dataset(range(3000, 3002), duration_s=30.0)
    X = X[:100]
    keras_mu, keras_lv = KerasSpeedEstimator().estimate_batch(X)
    tfl = TfliteSpeedEstimator()
    tfl_mu, tfl_lv = tfl.estimate_batch(X)
    assert np.mean(np.abs(keras_mu - tfl_mu)) < 0.05
    assert np.mean(np.abs(keras_lv - tfl_lv)) < 0.1
    tfl.estimate(X[0])
    t0 = time.perf_counter()
    for w in X[:50]:
        tfl.estimate(w)
    assert (time.perf_counter() - t0) / 50 < 0.005          # < 5 ms per window, batch size 1


def test_model_card_is_honest_and_complete():
    card = json.loads((MODELS_DIR / "model_card.json").read_text())
    assert card["label"] == "SYNTHETIC" and card["trainable_params"] <= 50_000
    for key in ("val", "normalisation", "data_sources", "limitations", "tflite_size_kb", "splits"):
        assert key in card
