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
- [ ] geo (psi convention, round-trip < 1 mm)
- [ ] simulator (seeded, phone-grade IMU, GNSS, outages, spoof step/ramp)
- [ ] metrics (hand-computed unit tests)
- [ ] 3 fixtures + `scripts/cut_fixture.py`

## P2-P10
Not started. Web (P8) and Android (P9) also wait on `design/stitch/`.
