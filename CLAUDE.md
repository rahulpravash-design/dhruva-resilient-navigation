# DHRUVA

Phone-only AI dead-reckoning navigation engine. 48-hour SIH build.
Android app (Kotlin/Compose/MapLibre) + web dashboard (React/Vite) + spoofing detection + offline maps/routing + measured drift metrics.

## Roles
A fusion lead · B ML · C Android · D data/map · E eval/dashboard · F pitch/design

## Repo layout
```
engine_py/     Python prototype (EKF, baseline, evaluation, injectors)
android/       Kotlin/Compose app, Kotlin port of the engine
dashboard/     React + Vite + TypeScript + Tailwind + Zustand + Zod + uPlot + MapLibre + PMTiles
data/          GSDC, IO-VNBD, own GnssLogger drives (split by trajectory, never by row)
docs/          CONTRACT.md (frozen at Hour 2), metrics, evidence
design/stitch/ Stitch screens + _claude_code_handoff/ (prompt, content audit, tokens, contracts)
runs/<name>/   timeline.jsonl + metrics.json per evaluation run
```
Create folders only when the current phase needs them.

## Engine contract (frozen at Hour 2 — change only with team sign-off)
INPUT per row:
`t, ax, ay, az, gx, gy, gz, mx, my, mz, gnss_lat, gnss_lon, gnss_acc, gnss_speed, gnss_bearing, cn0_mean, n_sats, gnss_valid`

OUTPUT per row:
`t, lat, lon, cov_ee, cov_en, cov_nn, speed, heading, mode {GNSS|DR|DEGRADED}, gnss_trust (0–1)`

Python and Kotlin engines implement this same contract. `NavState` (`contracts/nav_state.ts`) is the single engine→UI contract.

## Engine design
- Error-state EKF: position E/N, velocity, heading, gyro bias.
- Non-holonomic constraints: no sideways/vertical velocity.
- Mount alignment: roll/pitch from gravity, yaw from PCA of acceleration on straight segments.
- Speed net: 1D-CNN or GRU, <50k params, TFLite, used as EKF pseudo-measurement.
- GNSS integrity: NIS innovation test, C/N0, sat count, accuracy, hysteresis, 2 s blend on re-entry.
- HMM map-matching; A* routing over OSM-derived graph JSON.
- Ablation flags: `--no-ml --no-nhc --no-map --no-integrity`.
- Injectors: outage 30/60/120 s; spoof = 500 m sudden offset and 2 m/s slow drift.

## Metrics
Endpoint error, max error, drift % of distance, 95%-ellipse coverage, re-entry jump size.
Report every one per run. Never report a single cherry-picked number.

## Honesty rules (non-negotiable)
1. Measured value → show the number. Not measured → show `— NOT MEASURED`. Mock data → label `SIMULATED`.
2. Never mix phone-only and vehicle-grade results in one table, chart, or sentence.
3. PASS/FAIL only against a stated PS benchmark; otherwise `— NOT MEASURED`.
4. Never present these Stitch placeholder claims as fact: "0.84% verified on Pixel 7", "48.6 km Delhi-NCR", "NovAtel SPAN RTK", "ISO 26262 AUDIT", "$4.2B lost annually", "28 NavIC", "Secure Enclave".
5. Every displayed metric must trace to a run (SHA-256 of the source `timeline.jsonl`/`metrics.json`).
6. No train/test leakage: split by trajectory.

## Phase gates
| Phase | Gate | Must be true |
|---|---|---|
| −1 Pre-work | — | datasets + tooling ready |
| 0 Contract | H2 | `docs/CONTRACT.md` frozen |
| 1 Baseline | H8 | physics DR drift % measured |
| 2 Core engine | H16 | EKF+NHC beats baseline |
| 3 Intelligence | H24 | ablation v1 done, spoof rejection works |
| 4 On-device | H32 | **feature freeze**; Kotlin/Python parity within 0.5 m |
| 5 Evidence/polish | H40 | numbers + dashboard final |
| 6 Pitch | — | demo rehearsed |
| 7 Buffer | — | tag `v1.0-sih` |

Do not start a phase before the previous gate is met.

## Cut order if behind
1. On-device rerouting
2. On-device map-matching
3. Live spoof demo
4. Last resort: Kotlin EKF → laptop replay. Say so honestly to the judges.

## UI rules
- Data sources: Replay (`runs/<name>/…`), Live, Mock (always labelled `SIMULATED`).
- Fully offline: no CDN fonts, icons, CSS, or map tiles.
- Never rely on colour alone for status; add GOOD / WARNING / CRITICAL text.
- Stitch has 2 duplicate screen pairs — pick one canonical version each and record which.
- Work phase by phase, stop for review between phases, commit per phase:
  P0 inventory only (no building) → P1 components + `/kitchen-sink` → P2 screens on mock data + Playwright screenshot diff vs Stitch → P3 replay/real data + SHA-256 traceability → P4 Android → P5 offline + accessibility + honesty sweep.
- Design tokens come from `_claude_code_handoff/tokens/` — don't hand-pick colours.

## Working rules (from global CLAUDE.md)
- Plan first; if a task touches more than ~3 files, outline before building.
- Targeted edits; don't rewrite files.
- Never touch `.env`, secrets, or credentials without asking.
- Never `git push --force` without explicit confirmation.
- Never delete files outside the current task's scope.
- Define a success check for each task and loop until it passes; if unsure, say so.
