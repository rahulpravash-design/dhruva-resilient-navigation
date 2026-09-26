"""Physics-only dead-reckoning baseline: no EKF, no ML, no map.

Uses each GNSS fix directly when one arrives. Between fixes it integrates the gyro
for heading and the forward accelerometer for speed. Assumes the phone is flat and
aligned with the vehicle (x forward, z up); alignment is Phase 2 work.
"""
import numpy as np

from ..geo import to_enu, to_latlon
from .contract import validate_input, validate_output

FIX_TIMEOUT_S = 2.0   # rows within this of the last fix count as mode GNSS
SIGMA_GROWTH = 0.5    # m of 1-sigma per second without a fix (heuristic, not calibrated)


def run(trace):
    validate_input(trace)
    t = np.asarray(trace["t"], float)
    n = len(t)
    valid = trace["gnss_valid"]
    lat0, lon0 = trace["gnss_lat"][0], trace["gnss_lon"][0]
    fe, fn = to_enu(np.where(valid, trace["gnss_lat"], lat0),
                    np.where(valid, trace["gnss_lon"], lon0), lat0, lon0)

    e = np.zeros(n)
    nn = np.zeros(n)
    spd = np.zeros(n)
    hdg = np.zeros(n)
    sigma = np.zeros(n)
    mode = np.empty(n, dtype="<U8")
    trust = np.zeros(n)

    h = _bearing_rad(trace["gnss_bearing"][0], 0.0)
    v = float(trace["gnss_speed"][0])
    pe, pn = 0.0, 0.0
    last_fix_t = t[0]
    acc = float(trace["gnss_acc"][0])

    for i in range(n):
        if i > 0:
            dt = t[i] - t[i - 1]
            h -= trace["gz"][i] * dt
            v = max(0.0, v + trace["ax"][i] * dt)
            pe += v * np.sin(h) * dt
            pn += v * np.cos(h) * dt
        if valid[i]:
            pe, pn = fe[i], fn[i]
            v = float(trace["gnss_speed"][i])
            if v > 1.0:
                h = _bearing_rad(trace["gnss_bearing"][i], h)
            last_fix_t = t[i]
            acc = float(trace["gnss_acc"][i])
        since = t[i] - last_fix_t
        e[i], nn[i], spd[i], hdg[i] = pe, pn, v, np.degrees(h) % 360
        sigma[i] = acc + SIGMA_GROWTH * since
        mode[i] = "GNSS" if since <= FIX_TIMEOUT_S else "DR"
        trust[i] = 1.0 if mode[i] == "GNSS" else 0.0

    lat, lon = to_latlon(e, nn, lat0, lon0)
    out = {
        "t": t, "lat": lat, "lon": lon,
        "cov_ee": sigma ** 2, "cov_en": np.zeros(n), "cov_nn": sigma ** 2,
        "speed": spd, "heading": hdg, "mode": mode, "gnss_trust": trust,
    }
    validate_output(out, n_expected=n)
    return out


def _bearing_rad(bearing_deg, fallback_rad):
    return fallback_rad if not np.isfinite(bearing_deg) else np.radians(bearing_deg)


if __name__ == "__main__":
    from .inject import outage
    from .metrics import evaluate
    from .synth import make_drive

    print("Physics DR baseline on a SIMULATED drive (not real-world evidence)")
    print(f"{'outage':>7} {'endpoint_m':>11} {'max_m':>9} {'drift_%':>9} {'cov95':>7} {'jump_m':>9}")
    for dur in (30, 60, 120):
        trace, truth = make_drive(duration=400.0, seed=0)
        start = trace["t"][0] + 150.0
        est = run(outage(trace, start, dur))
        win = (trace["t"] >= start) & (trace["t"] < start + dur)
        m = evaluate(est, truth, mask=win)
        print(f"{dur:>6}s {m['endpoint_error_m']:>11.1f} {m['max_error_m']:>9.1f} "
              f"{m['drift_pct']:>9.2f} {m['coverage_95']:>7.2f} {m['reentry_jump_m']:>9.1f}")
