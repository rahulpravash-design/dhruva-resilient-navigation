"""Train SpeedNet on SYNTHETIC drives, export Keras + float16 TFLite + model_card.json.

usage: python -m dhruva.speednet.train [--epochs N] [--train-drives N]
"""
import argparse
import json
import time
from pathlib import Path

import keras
import numpy as np
import tensorflow as tf

from .data import make_dataset, split_seeds
from .infer import MODELS_DIR
from .model import build_model, gaussian_nll, trainable_params

DATA_SOURCES = ["SYNTHETIC: dhruva.sim with ASSUMED speed-dependent vibration (see docs/SPEEDNET_ASSUMPTIONS.md)"]


def export_tflite(model, path):
    conv = tf.lite.TFLiteConverter.from_keras_model(model)
    conv.optimizations = [tf.lite.Optimize.DEFAULT]
    conv.target_spec.supported_types = [tf.float16]
    blob = conv.convert()
    Path(path).write_bytes(blob)
    return len(blob)


def evaluate(model, X, y):
    out = model.predict(X, verbose=0, batch_size=1024)
    mu, lv = out[:, 0], out[:, 1]
    rmse = float(np.sqrt(np.mean((mu - y) ** 2)))
    nll = float(np.mean(0.5 * (lv + (y - mu) ** 2 * np.exp(-lv))))
    z = np.abs(y - mu) / np.exp(0.5 * lv)
    return {"rmse_mps": rmse, "nll": nll, "coverage_1sigma": float(np.mean(z <= 1.0)),
            "coverage_2sigma": float(np.mean(z <= 2.0))}


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--epochs", type=int, default=40)
    ap.add_argument("--train-drives", type=int, default=None)
    ap.add_argument("--out", default=str(MODELS_DIR))
    args = ap.parse_args(argv)
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    keras.utils.set_random_seed(0)

    t0 = time.time()
    train_seeds = split_seeds("train")[: args.train_drives]
    Xtr, ytr = make_dataset(train_seeds)
    Xva, yva = make_dataset(split_seeds("val"))
    print(f"data: train {Xtr.shape} val {Xva.shape} in {time.time() - t0:.0f}s", flush=True)

    mean = Xtr.reshape(-1, Xtr.shape[-1]).mean(0)
    std = Xtr.reshape(-1, Xtr.shape[-1]).std(0) + 1e-6
    model = build_model(mean, std)
    model.compile(optimizer=keras.optimizers.Adam(2e-3), loss=gaussian_nll)
    cbs = [keras.callbacks.EarlyStopping(patience=6, restore_best_weights=True, monitor="val_loss"),
           keras.callbacks.ReduceLROnPlateau(factor=0.5, patience=3, monitor="val_loss")]
    hist = model.fit(Xtr, ytr, validation_data=(Xva, yva), epochs=args.epochs, batch_size=256, callbacks=cbs,
                     verbose=2)

    val = evaluate(model, Xva, yva)
    model.save(out / "speednet.keras")
    size = export_tflite(model, out / "speednet.tflite")
    card = {
        "name": "speednet", "label": "SYNTHETIC",
        "trainable_params": trainable_params(model), "tflite_size_kb": round(size / 1024, 1),
        "tflite_precision": "float16 weights",
        "window": {"samples": int(Xtr.shape[1]), "rate_hz": 100, "features": [
            "ax", "ay", "az-g", "gx", "gy", "gz", "|a_h|"], "frame": "levelled vehicle frame"},
        "normalisation": {"mean": mean.tolist(), "std": std.tolist(), "embedded_in_model": True},
        "data_sources": DATA_SOURCES,
        "splits": {"train_drives": len(train_seeds), "val_drives": len(split_seeds("val")),
                   "train_windows": len(Xtr), "val_windows": len(Xva)},
        "val": val, "epochs_run": len(hist.history["loss"]),
        "limitations": ["Trained only on simulated drives with an assumed vibration model: no real-world performance claim.",
                        "Input must be resampled to 100 Hz and levelled by the aligner."],
    }
    (out / "model_card.json").write_text(json.dumps(card, indent=2) + "\n")
    print(json.dumps(card["val"], indent=2), f"\nparams {card['trainable_params']}, tflite {card['tflite_size_kb']} KB")


if __name__ == "__main__":
    main()
