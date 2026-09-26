"""Ground-truth vehicle trajectory. ENU frame, yaw psi CCW from East, vehicle frame x fwd / y left / z up."""
from dataclasses import dataclass

import numpy as np

DT = 0.01  # 100 Hz


@dataclass(frozen=True)
class Segment:
    kind: str              # "straight" | "turn" | "stop"
    dur: float             # seconds
    speed: float = 0.0     # target speed, m/s (ignored for "stop")
    turn_deg: float = 0.0  # total heading change over the segment, CCW positive ("turn" only)


def _lowpass(target, tau, x0):
    a = DT / (tau + DT)
    out = np.empty_like(target)
    x = x0
    for k, u in enumerate(target):
        x += a * (u - x)
        out[k] = x
    return out


def make_trajectory(segments, psi0=0.3, accel_max=1.5, brake_max=2.5):
    """Return dict of per-row truth: t, e, n, v, psi, omega, a_long (all float64)."""
    n = sum(round(s.dur / DT) for s in segments)
    v_cmd = np.empty(n)
    w_cmd = np.zeros(n)
    k = 0
    v_prev = segments[0].speed
    for s in segments:
        m = round(s.dur / DT)
        target = 0.0 if s.kind == "stop" else s.speed
        rate = np.radians(s.turn_deg) / s.dur if s.kind == "turn" else 0.0
        for j in range(m):
            step = accel_max * DT if target > v_prev else brake_max * DT
            v_prev += np.clip(target - v_prev, -step, step)
            v_cmd[k + j] = v_prev
            w_cmd[k + j] = rate
        k += m
    v = _lowpass(v_cmd, 0.4, v_cmd[0])
    omega = _lowpass(w_cmd, 0.5, 0.0)

    psi = np.empty(n)
    e = np.zeros(n)
    nn = np.zeros(n)
    psi[0] = psi0
    for i in range(n - 1):
        psi[i + 1] = psi[i] + omega[i] * DT
        e[i + 1] = e[i] + v[i] * np.cos(psi[i]) * DT
        nn[i + 1] = nn[i] + v[i] * np.sin(psi[i]) * DT
    a_long = np.empty(n)
    a_long[:-1] = np.diff(v) / DT
    a_long[-1] = a_long[-2]
    return {"t": np.arange(n) * DT, "e": e, "n": nn, "v": v, "psi": psi, "omega": omega, "a_long": a_long}
