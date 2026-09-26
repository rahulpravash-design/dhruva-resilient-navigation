# DHRUVA architecture (as implemented)

DHRUVA does not replace GNSS. It keeps a position estimate going when GNSS is unavailable, degraded or untrustworthy.
This file describes what exists in the repository today. Status tags: **IMPLEMENTED**, **SIMULATED** (only exercised on
synthetic data), **NOT WIRED** (code exists but the engine does not use it), **PLANNED** (does not exist).

## Data flow

```
fixture CSV / replay  (SIMULATED today; no live sensors)
        |
        v   IMU 100 Hz + GNSS ~1 Hz
   MountAligner  ---- phone frame -> vehicle frame (gravity + yaw)            IMPLEMENTED
        |
        v
   Error-state EKF (8 states: pE pN vE vN psi b_g b_ax b_ay)                   IMPLEMENTED
     + GNSS position / velocity / bearing updates
     + NHC (no sideways / vertical velocity), ZUPT, ZARU
     + SpeedNet speed pseudo-measurement (TFLite, optional)                     IMPLEMENTED, trained on SYNTHETIC data
        |
        v
   IntegrityMonitor  (gates GNSS updates: NIS, CUSUM, re-entry validation)      IMPLEMENTED, SIMULATED spoofs only
        |
        v   10 Hz
   timeline row -> NavState (contracts/nav_state.schema.json)                  IMPLEMENTED
        |
        v
   web/index.html  (offline replay dashboard)                                   IMPLEMENTED
   Android app, Kotlin engine                                                   PLANNED
   HMM map matcher + A* router (dhruva/mapmatch, dhruva/route)                  NOT WIRED (tested on synthetic graphs)
```

## Engine modules (`engine_py/dhruva/`)
| Module | Role |
|---|---|
| `geo.py` | the only place lat/lon <-> ENU and heading/yaw conversions happen |
| `config.py`, `configs/engine.json` | single source of engine parameters |
| `io/fixture.py` | fixture CSV reader/writer (21 columns) |
| `sim/` | seeded drive simulator with an assumed phone IMU/GNSS model; `faults.py` injects outages and spoofs |
| `align/` | `MountAligner` (gravity + straight-line yaw) and `OracleAligner` (true mount, for tests) |
| `ekf/` | `Engine` (full filter), `BaselineEngine` (gyro heading + held GNSS speed), shared `EngineBase`, `EngineFlags` for ablations |
| `speednet/` | feature windows, 1D-CNN model, training, TFLite inference |
| `integrity/` | `IntegrityMonitor`: LOCK / DR / REACQUIRE / SPOOF_REJECTED, `gnss_trust` 0 to 1 |
| `mapmatch/`, `route/` | HMM/Viterbi matcher and A* router with deviation detection (NOT WIRED into `Engine`) |
| `metrics/` | endpoint error, drift %, coverage, re-entry jump, PS-benchmark check |
| `eval/` | replay helpers, evaluation matrix runner and RESULTS.md report |
| `export/` | NavState records for the UIs |
| `legacy/` | superseded flat prototype (kept, not extended) |

## Conventions
ENU navigation frame at the first fix; vehicle body x forward, y left, z up; internal yaw psi in radians counter-clockwise
from East; UI heading `(90 - psi deg) mod 360`; SI units; sensor time only. See `docs/CONTRACT.md` for the legacy contract
and `contracts/nav_state.schema.json` for the current output record.

## Modes shown to a user
| Engine state | NavState mode / gnssStatus |
|---|---|
| GNSS fixes accepted | `GNSS` / `OK` (trust >= 0.5) |
| no fixes | `DR` / `UNAVAILABLE` |
| DR while trust climbs back | `DR` / `VALIDATING` |
| fixes rejected as inconsistent | `DR` / `REJECTED` |
| mount alignment not confident | `DEGRADED` |

## What is not built
Android app, Kotlin engine and Kotlin/Python parity check, live sensor capture, MapLibre and offline tiles, map matching
inside the engine, any real dataset. See `docs/LIMITATIONS.md`.
