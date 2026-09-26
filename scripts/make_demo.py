"""Export the SIMULATED demo scenarios for the offline dashboard (web/data/demo.js).

Runs the real DHRUVA engine (EKF + NHC + SpeedNet + integrity monitor) and the gyro-heading baseline on one seeded
SYNTHETIC drive: the first `test` split drive, not chosen by result. Scenarios: 60 s GNSS outage, 500 m step spoof,
2 m/s ramp spoof. Everything in the output is labelled SIMULATED. Deterministic: same seed, same file.

    .venv\\Scripts\\python scripts\\make_demo.py
"""
import hashlib
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "engine_py"))

from dhruva.config import DEFAULT_PATH, load_config
from dhruva.eval.runner import _clean, _run
from dhruva.geo import psi_to_heading_deg, to_enu
from dhruva.metrics import outage_metrics, reentry_jump_m, time_to_detect_s
from dhruva.sim import apply_outage, apply_spoof_ramp, apply_spoof_step, simulate
from dhruva.speednet.data import random_scenario

SEED, DURATION_S = 4000, 135.0
OUTAGE_T0, OUTAGE_S = 60.0, 60.0
SPOOF_T0, SPOOF_DURATION_S = 60.0, 100.0
MODES = ["GNSS", "DR", "DEGRADED", "REACQUIRE", "SPOOF_REJECTED"]
OUT = ROOT / "web" / "data" / "demo.js"


def _r(a, nd):
    return [None if not np.isfinite(x) else round(float(x), nd) for x in np.asarray(a, float)]


def _enu(lat, lon, origin):
    e, n = to_enu(np.asarray(lat, float), np.asarray(lon, float), *origin)
    return np.asarray(e), np.asarray(n)


def _scenario(sid, title, df, cfg, kind, t0, t1=None):
    tl, _ = _run(df, "integrity", cfg)
    bl, _ = _run(df, "baseline", cfg)
    origin = (float(tl["lat"].iloc[0]), float(tl["lon"].iloc[0]))
    e, n = _enu(tl["lat"], tl["lon"], origin)
    te, tn = _enu(tl["truth_lat"], tl["truth_lon"], origin)
    be, bn = _enu(np.interp(tl["t"], bl["t"], bl["lat"]), np.interp(tl["t"], bl["t"], bl["lon"]), origin)
    fx = df[df["gnss_new"] & np.isfinite(df["gnss_lat"])]
    fe, fn = _enu(fx["gnss_lat"], fx["gnss_lon"], origin)
    t = tl["t"].to_numpy()
    tick = slice(None, None, 2)  # 10 Hz engine ticks -> 5 Hz in the file
    heading = [psi_to_heading_deg(p) for p in tl["psi"]]
    mode_idx = [MODES.index(m) for m in tl["mode"]]

    def metrics(x):
        idx = np.clip(np.searchsorted(t, x["t"].to_numpy()), 0, len(t) - 1)
        if kind == "outage":
            m = outage_metrics(x["t"], x["lat"], x["lon"], x["cov95_m"], tl["truth_lat"].to_numpy()[idx],
                               tl["truth_lon"].to_numpy()[idx], t0, t1 - 0.02)
            m["reentry_jump_m"] = reentry_jump_m(x["t"], x["lat"], x["lon"], x["speed"], t1)
            return m
        xe, xn = _enu(x["lat"], x["lon"], origin)
        err = np.hypot(xe - te[idx], xn - tn[idx])
        after = x["t"].to_numpy() >= t0
        return {"max_err_after_m": float(err[after].max()), "final_err_m": float(err[-1])}

    m_d, m_b = metrics(tl), metrics(bl)
    if kind == "outage":
        m_d["time_to_dr_s"] = time_to_detect_s(tl["t"], (tl["mode"] != "GNSS").to_numpy(), t0)
    else:
        m_d["time_to_detect_s"] = time_to_detect_s(tl["t"], (tl["mode"] == "SPOOF_REJECTED").to_numpy(), t0)
    return _clean({
        "id": sid, "title": title, "kind": kind, "label": "SIMULATED", "t0": t0, "t1": t1,
        "duration_s": float(t[-1]), "modes": MODES,
        "t": _r(t[tick], 2), "e": _r(e[tick], 1), "n": _r(n[tick], 1),
        "truth_e": _r(te[tick], 1), "truth_n": _r(tn[tick], 1), "base_e": _r(be[tick], 1), "base_n": _r(bn[tick], 1),
        "speed_kmh": _r((tl["speed"].to_numpy() * 3.6)[tick], 1), "heading": _r(np.asarray(heading)[tick], 1),
        "cov95_m": _r(tl["cov95_m"].to_numpy()[tick], 1), "trust": _r(tl["gnss_trust"].to_numpy()[tick], 2),
        "mode": [mode_idx[i] for i in range(0, len(mode_idx), 2)],
        "gnss": {"t": _r(fx["t"], 2), "e": _r(fe, 1), "n": _r(fn, 1)},
        "metrics": {"dhruva": m_d, "baseline": m_b},
    })


def main():
    cfg = load_config()
    sim = simulate(random_scenario(SEED, DURATION_S))
    v = np.interp([OUTAGE_T0, OUTAGE_T0 + OUTAGE_S], sim.truth["t"], sim.truth["v"])
    print(f"truth speed at outage start/end: {v[0]:.1f} / {v[1]:.1f} m/s")
    scenarios = [_scenario("outage", "GNSS outage, 60 s", apply_outage(sim.df, OUTAGE_T0, OUTAGE_S), cfg, "outage",
                           OUTAGE_T0, OUTAGE_T0 + OUTAGE_S)]
    base = sim.df[sim.df["t"] <= SPOOF_DURATION_S].reset_index(drop=True)
    scenarios.append(_scenario("spoof_step", "SIMULATED SPOOF SCENARIO: 500 m step",
                               apply_spoof_step(base, SPOOF_T0, 500.0), cfg, "spoof", SPOOF_T0))
    scenarios.append(_scenario("spoof_ramp", "SIMULATED SPOOF SCENARIO: 2 m/s drift",
                               apply_spoof_ramp(base, SPOOF_T0, 2.0), cfg, "spoof", SPOOF_T0))
    demo = {"label": "SIMULATED", "generator": "scripts/make_demo.py", "seed": SEED, "split": "test",
            "grade": "PHONE", "drive_duration_s": DURATION_S,
            "engine_json_sha256": hashlib.sha256(DEFAULT_PATH.read_bytes()).hexdigest(), "scenarios": scenarios}
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text("window.DHRUVA_DEMO = " + json.dumps(demo, separators=(",", ":")) + ";\n", encoding="utf-8",
                   newline="\n")
    print(f"wrote {OUT.relative_to(ROOT)} ({OUT.stat().st_size // 1024} KiB)")
    for s in scenarios:
        print(s["id"], json.dumps(s["metrics"]))


if __name__ == "__main__":
    main()
