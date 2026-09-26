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

## P2-P10
Not started. Web (P8) and Android (P9) also wait on `design/stitch/`.
