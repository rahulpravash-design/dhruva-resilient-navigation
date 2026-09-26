# Blockers

## B1: design/stitch/ and _claude_code_handoff/ missing (open)
- Symptom: no `contracts/nav_state.ts`, tokens, `CONTENT_AUDIT.md` or `CLAUDE_CODE_PROMPT.md` on disk.
- Affects: P0 schema generation, Python output schema validation, Kotlin `NavState` mirror, all of P8/P9.
- Not invented: `NavState` is not guessed. Engine work proceeds on the fixture format until the file lands.
- Needed from user: copy `design/stitch/` into the repo.
- Also to confirm when unblocked: which npm tool generates the JSON Schema from `nav_state.ts` (not named in the master prompt, so it needs approval).

## B2: Android SDK not installed (deferred by choice)
- Needed for P9 only. JDK 17 and Gradle 8.10.2 are present for the pure-JVM `engine_kt` (P7).

## B3: P3 gate "aligned within 10 % of perfect-mount run" not met (open, non-blocking)
- Symptom: aligned EKF endpoint error 2.75 m vs 1.71 m with the true mount (6 seeds, long synthetic drive, 16 s outage).
- Tried: gravity gyro-bias estimate; decoupled longitudinal/lateral yaw evidence; mean-removed sums instead of high-pass;
  skip accelerometer propagation until confident; fast acceleration-onset gate; freeze accel correction during outage.
  Hybrid test: estimated yaw rate is harmless; the residual comes from the levelled horizontal force (tilt error 0.3-0.8 deg).
- Hypothesis: a tilt/leak state in the EKF or GNSS-aided tilt refinement would close the gap; SpeedNet (P4) also bounds the
  forward-speed error that the leak causes. Deferred rather than over-tuned on the simulator.

## B4: engine uncertainty overconfident / heavy-tailed failures on random drives (largely resolved)
- Symptom (first P4 eval, SYNTHETIC): median 95 %-circle coverage 0-12 %, p90 endpoint error 250-400 m.
- Root cause found by tracing the worst seeds: EKF accel process noise (0.01) and accel-bias random walk (5e-4) were far too small
  for the constant 0.3-0.6 m/s^2 horizontal offset a 2-3 deg tilt error leaks in. The filter trusted its own velocity, ignored
  GNSS velocity innovations (NIS ~200), and drifted 3-5 m/s off while cov95 read 3 m. A ZUPT deadlock was suspected first
  and ruled out (identical output after the fix); the stricter stationary detector (steady accel vector + GNSS veto) was kept.
- Fix: retuned on the validation split only (accel noise 0.15, bias RW 5e-3). Test split: p90 endpoint error 36 m (30 s) / 65 m (60 s)
  without SpeedNet; median coverage 94-100 % without SpeedNet.
- Remaining: tails (see B5) and SpeedNet-on coverage 69 % at 60 s.

## B5: alignment quality is not represented in the filter (open)
- Symptom: some drives end with 3-6 deg tilt and 5-11 deg yaw error while `confident` is True; those give the remaining
  worst outage errors (val seeds 2015, 2019, 2025).
- Idea: turn the LS coherence / residual into a quality number that inflates accel noise, and refine tilt with GNSS-aided velocity.
