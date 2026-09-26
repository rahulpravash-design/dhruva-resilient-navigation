# Testing

## Commands (from the repo root, inside the virtualenv)
```
python -m ruff check engine_py scripts
python -m pytest engine_py/tests -m "not slow" -n auto -q      # fast tier, 147 passed at the last run
python -m pytest -m golden -q                                    # golden-output tests
python -m pytest -m slow -q                                      # slow tier
```
`make test-fast`, `make test-golden` and `make test-full` wrap the same commands (GNU make is needed on Windows).
`engine_py` is put on the Python path by `pyproject.toml`. The web page has no JavaScript test setup; it is checked by
running it (see below).

## What the tests cover (`engine_py/tests/`)
| File | Covers |
|---|---|
| `test_geo.py` | lat/lon <-> ENU and heading/yaw conversions |
| `test_sim.py`, `test_fixtures.py` | simulator determinism, outage/spoof injectors, fixture format |
| `test_metrics_core.py` | endpoint error, drift %, coverage, re-entry jump, PS-benchmark rule |
| `test_align.py` | mount alignment (gravity, yaw, realignment) |
| `test_ekf.py`, `golden/` | filter behaviour on the 3 SYNTHETIC fixtures; golden timelines and logs |
| `test_speednet_model.py`, `test_speednet_engine.py` | SpeedNet model, TFLite inference, engine pseudo-measurement |
| `test_integrity_monitor.py`, `test_integrity_engine.py` | integrity state machine, spoof rejection, safe re-entry |
| `test_map_route.py` | HMM map matching and A* routing on SYNTHETIC graphs |
| `test_navstate.py` | NavState records validate against `contracts/nav_state.schema.json` |
| `test_eval_report.py` | RESULTS.md rendering is deterministic and labelled |
| `test_demo_data.py` | `web/data/demo.js` is labelled SIMULATED and consistent; web page and README have no banned claims or external URLs |
| `test_layout.py` | package layout; engine.json holds the spec values (NHC sigma, chi-square threshold, PS benchmark) |
| `legacy/` | the superseded flat prototype |

## Rules
- Golden outputs are regenerated only with `make golden-update` and the commit message states why.
- Tolerances are never widened to make a test pass.
- Split by trajectory: train 1000-1149, val 2000-2029, dev 3000-3039 (contaminated by tuning), test 4000-4039 (never tuned on).
- A bug fix gets a regression test.

## Manual demo check (the web page)
1. `python -m http.server 8765 --directory web`, open http://localhost:8765/index.html.
2. Press **[ START DEMO ]**. Expect GNSS MODE, DEAD RECKONING at about 60 s, GNSS RECOVERY at about 121 s, GNSS MODE at about 122 s.
3. Pick each spoof scenario on the Replay tab; expect "GNSS REJECTED: DEAD RECKONING" after the spoof starts at 60 s.
4. Browser console: no errors or warnings.

## Not covered
Kotlin/Python parity (no Kotlin engine), Android instrumentation, JavaScript unit tests, real-data regression.
