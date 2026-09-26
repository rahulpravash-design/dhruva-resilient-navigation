"""Phone-grade IMU model: random mount rotation, scale factor, bias random walk, white noise."""
from dataclasses import dataclass

import numpy as np

G = 9.80665
WHEEL_RADIUS_M = 0.31


@dataclass(frozen=True)
class ImuParams:
    gyro_noise_std: float = 0.003   # rad/s per sample
    accel_noise_std: float = 0.05   # m/s^2 per sample
    gyro_bias_std: float = 0.004    # rad/s, initial turn-on bias
    gyro_bias_rw: float = 2e-5      # rad/s/sqrt(s)
    accel_bias_std: float = 0.05    # m/s^2
    accel_bias_rw: float = 5e-4     # m/s^2/sqrt(s)
    scale_std: float = 0.005        # multiplicative scale-factor error
    # ASSUMED road-vibration model (not measured): accel vibration in the vehicle frame whose amplitude grows with
    # speed, white plus a tone at the wheel-rotation rate. 0 disables it. See docs/SPEEDNET_ASSUMPTIONS.md.
    vib_gain: float = 0.0           # m/s^2 of vibration std per m/s of speed

    @classmethod
    def zero(cls):
        return cls(0, 0, 0, 0, 0, 0, 0)


def _rz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


def _rx(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1.0, 0, 0], [0, c, -s], [0, s, c]])


def _ry(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s], [0, 1.0, 0], [-s, 0, c]])


def random_mount(rng, max_tilt_deg=60.0):
    """R_pv (phone <- vehicle): random yaw, roll/pitch within +-max_tilt_deg."""
    yaw = rng.uniform(-np.pi, np.pi)
    roll = np.radians(rng.uniform(-max_tilt_deg, max_tilt_deg))
    pitch = np.radians(rng.uniform(-max_tilt_deg, max_tilt_deg))
    return _rz(yaw) @ _rx(roll) @ _ry(pitch)


def _rot_axis(axis, theta):
    """Rodrigues rotation matrices (n, 3, 3) about a fixed unit axis for angles theta (n,)."""
    a = np.asarray(axis, float) / np.linalg.norm(axis)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    th = np.asarray(theta)[:, None, None]
    return np.eye(3) + np.sin(th) * K + (1 - np.cos(th)) * (K @ K)


def mount_after_move(R_pv, move):
    """Final R_pv after a phone move (t_start, dur_s, angle_deg, axis_in_vehicle_frame)."""
    return R_pv @ _rot_axis(move[3], [np.radians(move[2])])[0]


def phone_imu(traj, R_pv, params, rng, dt, move=None):
    """Truth (vehicle frame) -> phone-frame accel [m/s^2] and gyro [rad/s], each (n, 3).

    move: optional (t_start, dur_s, angle_deg, axis_in_vehicle_frame). The phone is rotated relative to the
    vehicle by a smoothstep angle about a vehicle-fixed axis; the gyro sees that rotation, so the phone-frame
    gravity direction stays consistent with it.
    """
    n = len(traj["t"])
    f_veh = np.stack([traj["a_long"], traj["v"] * traj["omega"], np.full(n, G)], axis=1)
    w_veh = np.stack([np.zeros(n), np.zeros(n), traj["omega"]], axis=1)
    if move is None:
        acc = f_veh @ R_pv.T
        gyr = w_veh @ R_pv.T
    else:
        t_start, dur, angle_deg, axis = move
        x = np.clip((traj["t"] - t_start) / dur, 0.0, 1.0)
        theta = np.radians(angle_deg) * (3 * x ** 2 - 2 * x ** 3)
        theta_dot = np.radians(angle_deg) * (6 * x - 6 * x ** 2) / dur * ((x > 0) & (x < 1))
        Rt = R_pv @ _rot_axis(axis, theta)                       # (n, 3, 3), phone <- vehicle over time
        a_u = np.asarray(axis, float) / np.linalg.norm(axis)
        acc = np.einsum("nij,nj->ni", Rt, f_veh)
        gyr = np.einsum("nij,nj->ni", Rt, w_veh) - theta_dot[:, None] * np.einsum("nij,j->ni", Rt, a_u)

    scale_a = 1.0 + rng.normal(0, params.scale_std, 3)
    scale_g = 1.0 + rng.normal(0, params.scale_std, 3)

    def bias(std, rw):
        b0 = rng.normal(0, std, 3)
        walk = np.cumsum(rng.normal(0, rw * np.sqrt(dt), (n, 3)), axis=0)
        return b0 + walk

    acc = acc * scale_a + bias(params.accel_bias_std, params.accel_bias_rw) \
        + rng.normal(0, params.accel_noise_std, (n, 3))
    gyr = gyr * scale_g + bias(params.gyro_bias_std, params.gyro_bias_rw) \
        + rng.normal(0, params.gyro_noise_std, (n, 3))

    if params.vib_gain > 0:   # drawn last so seeds without vibration are unchanged
        v = traj["v"]
        wheel_hz = v / (2 * np.pi * WHEEL_RADIUS_M)
        phase = 2 * np.pi * np.cumsum(wheel_hz) * dt + rng.uniform(0, 2 * np.pi)
        amp = params.vib_gain * v
        white = rng.normal(0.0, 1.0, (n, 3)) * amp[:, None] * np.array([0.4, 0.4, 1.0])
        tone = (0.8 * amp * np.sin(phase))[:, None] * np.array([0.3, 0.3, 1.0])
        vib_veh = white + tone
        acc = acc + (vib_veh @ R_pv.T if move is None else np.einsum("nij,nj->ni", Rt, vib_veh))
    return acc, gyr
