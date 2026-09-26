# Progress

Shell setup (each new shell on this machine):
`export PATH="$PATH:/c/Users/pravash.V/AppData/Local/Microsoft/WinGet/Packages/ezwinports.make_Microsoft.Winget.Source_8wekyb3d8bbwe/bin:/c/Users/pravash.V/tools/gradle-8.10.2/bin"`

## P0 Bootstrap
- [x] Layout, `pyproject.toml`, `requirements.txt`, `configs/engine.json`
- [x] Makefile (`setup`, `test-fast`, `test-golden`), CI workflow (not yet run: repo has no remote)
- [x] `docs/PROGRESS.md`, `BLOCKERS.md`, `DATA_STATUS.md`
- [x] Gate: `make test-fast` green on skeleton
- [ ] `contracts/nav_state.schema.json` — blocked: `nav_state.ts` not on disk (see BLOCKERS.md)

## P1 geo + simulator + metrics + fixtures
- [x] geo (psi convention, round-trip < 1 mm, heading table)
- [x] simulator (seeded, phone-grade IMU with random mount, GNSS 1 Hz, outages, spoof step/ramp)
- [x] metrics (hand-computed unit tests: outage, re-entry jump, time-to-detect, spoof stats, summarize)
- [x] 3 SYNTHETIC fixtures (3000 rows each) + `scripts/make_fixtures.py` + `scripts/cut_fixture.py`
- [x] Gate: metrics + sim tests green (`make test-fast`: 67 passed, ~5 s)
- Known gaps: `speednet_mu/logvar` columns are blank until P4; IMU timestamp jitter is not modelled;
  real-logger -> unified-fixture converter waits for real data (P6/P9).

## P2 baseline + EKF (+NHC, ZUPT/ZARU)
- [x] Baseline (gyro heading + held GNSS speed) and 8-state planar ES-EKF; oracle mount (true R_pv) until P3
- [x] Tests: zero-noise line < 0.1 m, left turn psi up, covariance grows/shrinks, NHC helps, ZUPT at stop, NIS consistency (slow), schema-free timeline checks
- [x] Golden outputs for the 3 fixtures (`make golden-update`, `make test-golden`)
- [x] Gate: EKF beats baseline on all 3 fixtures. SYNTHETIC endpoint error over the outage window:
  tunnel_straight 8.0 m vs 20.9 m; tunnel_turns 7.8 m vs 53.7 m; stop_go 2.6 m vs 6.1 m.
- Notes: (1) ZUPT is also gated on estimated speed < 1 m/s, because constant-velocity motion looks quasi-static to an IMU
  (first version zeroed speed at 16 m/s and produced 338 m error). (2) GNSS-pos NIS mean 1.5-1.8 vs DOF 2 (slightly conservative);
  NHC NIS ~0 because the simulator has no lateral slip; ZUPT NIS is high (~14) because ZUPT engages while still rolling <1 m/s.
  (3) 95%-circle coverage reads 100% in these runs, i.e. conservative, not yet a proof of calibration.
  (4) Engine state is a provisional `EngineState`; mapping to `NavState` waits for `nav_state.ts` (BLOCKERS B1).
  (5) Flags for ML/map/integrity/spoof are added in the phases that implement them.

## P3-P10
Not started. Web (P8) and Android (P9) also wait on `design/stitch/`.
