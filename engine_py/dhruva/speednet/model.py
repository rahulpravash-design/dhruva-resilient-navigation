"""SpeedNet: 1D-CNN, ~47k parameters. Input (150, 7) raw features, output [mu (m/s), logvar]."""
import keras
from keras import layers

from .features import N_FEATURES, WINDOW

LOGVAR_CLIP = (-6.0, 4.0)


class MuLogvar(layers.Layer):
    """mu = softplus(o0) (speed >= 0), logvar = clip(o1, -6, 4)."""

    def call(self, x):
        mu = keras.ops.softplus(x[:, 0:1])
        logvar = keras.ops.clip(x[:, 1:2], LOGVAR_CLIP[0], LOGVAR_CLIP[1])
        return keras.ops.concatenate([mu, logvar], axis=-1)


def build_model(mean=None, std=None):
    """mean/std: per-feature normalisation (embedded in the model so callers feed raw features)."""
    inp = keras.Input(shape=(WINDOW, N_FEATURES), name="window")
    x = inp
    if mean is not None:
        x = layers.Normalization(mean=mean, variance=std ** 2, name="norm")(x)
    for filters, k, stride in ((32, 7, 1), (64, 5, 2), (64, 5, 2), (64, 3, 2)):
        x = layers.Conv1D(filters, k, strides=stride, padding="same")(x)
        x = layers.BatchNormalization()(x)
        x = layers.ReLU()(x)
    x = layers.GlobalAveragePooling1D()(x)
    x = layers.Dense(32, activation="relu")(x)
    x = layers.Dense(2)(x)
    out = MuLogvar(name="mu_logvar")(x)
    return keras.Model(inp, out, name="speednet")


def gaussian_nll(y_true, y_pred):
    """0.5 * (logvar + (y - mu)^2 * exp(-logvar)); y_true (batch, 1) or (batch,)."""
    y = keras.ops.reshape(y_true, (-1, 1))
    mu, logvar = y_pred[:, 0:1], y_pred[:, 1:2]
    return keras.ops.mean(0.5 * (logvar + keras.ops.square(y - mu) * keras.ops.exp(-logvar)))


def trainable_params(model):
    return int(sum(int(keras.ops.prod(w.shape)) for w in model.trainable_weights))
