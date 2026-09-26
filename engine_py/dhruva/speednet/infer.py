"""SpeedEstimator implementations. All take a raw (150, 7) window and return (mu [m/s], logvar)."""
from pathlib import Path

import numpy as np

MODELS_DIR = Path(__file__).resolve().parents[3] / "models"


class KerasSpeedEstimator:
    def __init__(self, path=None):
        import keras

        from .model import MuLogvar
        self.model = keras.saving.load_model(path or MODELS_DIR / "speednet.keras", compile=False,
                                             custom_objects={"MuLogvar": MuLogvar})

    def estimate_batch(self, windows):
        out = self.model.predict(np.asarray(windows, np.float32), verbose=0, batch_size=1024)
        return out[:, 0], out[:, 1]

    def estimate(self, window):
        mu, lv = self.estimate_batch(np.asarray(window, np.float32)[None])
        return float(mu[0]), float(lv[0])


class TfliteSpeedEstimator:
    def __init__(self, path=None):
        import tensorflow as tf

        self.interp = tf.lite.Interpreter(model_path=str(path or MODELS_DIR / "speednet.tflite"))
        self.interp.allocate_tensors()
        self._in = self.interp.get_input_details()[0]["index"]
        self._out = self.interp.get_output_details()[0]["index"]

    def estimate(self, window):
        self.interp.set_tensor(self._in, np.asarray(window, np.float32)[None])
        self.interp.invoke()
        out = self.interp.get_tensor(self._out)[0]
        return float(out[0]), float(out[1])

    def estimate_batch(self, windows):
        res = np.array([self.estimate(w) for w in windows])
        return res[:, 0], res[:, 1]
