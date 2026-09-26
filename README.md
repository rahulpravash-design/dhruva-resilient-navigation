# DHRUVA — Resilient Navigation

**Navigation that does not stop where the sky does.** DHRUVA is a smartphone-only, AI-assisted dead-reckoning engine built for ISRO problem statement SIH26168. When GNSS is lost, degraded or untrustworthy, it keeps a position estimate going from the phone's own IMU, a small learned speed model, an error-state Kalman filter and motion constraints. It also checks GNSS integrity, so a returning or spoofed fix is validated before it is trusted.

> DHRUVA does not replace GNSS. DHRUVA provides resilient position estimation when GNSS becomes unavailable, degraded, or untrustworthy.

## Problem

Phone navigation fails in tunnels, under flyovers, in urban canyons, and when GNSS is jammed or spoofed. The PS success criterion is under 10 % drift and under 100 m error over 1 km at 60 km/h (about a 60 s outage), using only a smartphone.

## Solution

1. Align the phone's sensor frame to the vehicle (gravity gives roll and pitch, straight-line acceleration gives yaw).
2. Propagate position with an 8-state error-state EKF using the gyro and accelerometer.
3. Constrain drift: no sideways or vertical velocity (NHC), zero-velocity updates, and a SpeedNet 1D-CNN (TFLite, 47k parameters) as a speed measurement.
4. Watch GNSS with an integrity monitor (innovation tests, CUSUM, deadlock breaker). Reject inconsistent fixes, validate returning ones, and blend the display so the position does not jump when GNSS comes back.

## Demo

Everything runs offline. From the repo root on Windows:

```
.venv\Scripts\python -m http.server 8765 --directory web
```

Then open <http://localhost:8765/index.html>. Opening `web\index.html` directly also works. The demo data is committed in `web/data/demo.js`. To regenerate it from the real engine (about 1 minute):

```
.venv\Scripts\python scripts\make_demo.py
```

Three screens: **Navigation HUD** (mode, GNSS trust, speed, position confidence, trajectory), **Replay / GNSS Outage** (scenario picker, scrubber, error against simulated truth) and **Technical Metrics** (numbers for the run shown, plus everything that is NOT MEASURED). The map is a local trajectory view with no basemap and no network access.

## Demo Scenario

Press **[ SIMULATE GNSS OUTAGE ]**. The replay runs GNSS MODE (trust high) → DEAD RECKONING (trust low, simulated) → GNSS RECOVERY (validating GNSS) → GNSS MODE, with the uncertainty ring growing during the outage and the position recovering afterwards. Two simulated spoof scenarios (500 m step, 2 m/s drift) show the integrity monitor rejecting fixes. They cover those two attack types only and do not claim to detect all spoofing.

The demo drive is the first drive of the held-out `test` split (seed 4000), not chosen by result. On it, over a 60 s (1,007 m) outage, the engine ended 4.2 m off (0.42 % drift) against 81 m (8.05 %) for the gyro-heading baseline. This is one simulated drive, n = 1.

## Current Status

**Implemented (Python, tested):** error-state EKF, mount aligner, NHC/ZUPT, SpeedNet inference, integrity monitor, fault injectors (outage, step and ramp spoof), evaluation harness (`python -m dhruva.eval`), HMM map matcher and A* router (unit-tested on synthetic graphs, not wired into the engine), offline dashboard.

**Simulated:** every trajectory and sensor stream. The simulator uses an assumed phone IMU/GNSS model, and SpeedNet was trained on synthetic drives with an assumed vibration model (`docs/SPEEDNET_ASSUMPTIONS.md`).

**Pending validation / NOT MEASURED:** any real phone drive or public dataset (`data/raw/` is empty), vehicle-grade sensors, map-matched accuracy, on-device latency and battery, statistics over many drives (the harness exists; the full matrix has not been run), real-world spoof detection rate and false alarms. **Planned, not built:** Android app, Kotlin engine.

Known engine limits are in `docs/BLOCKERS.md` (for example a re-entry step above target on some drives, slow ramp spoofs taking about 15 to 19 s to reject, and a heavy tail of large errors on some outages).

## Important limitation

Simulated results are not field measurements. They show how the engine behaves inside the simulator's assumptions and are not evidence of real-world accuracy.

## Tests

```
.venv\Scripts\python -m ruff check engine_py scripts
.venv\Scripts\python -m pytest engine_py/tests -m "not slow" -n auto -q
```

## Team

| Person | Role |
|---|---|
| Person 1 | Backend |
| Person 2 | Frontend |
| Person 3 | Testing |
| Person 4 | UI/UX |

## Repository

https://github.com/rahulpravash-design/dhruva-resilient-navigation
