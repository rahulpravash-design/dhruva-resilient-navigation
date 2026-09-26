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

## B4: engine uncertainty is overconfident and has heavy-tailed failures on random drives (open)
- Symptom (P4 held-out eval, SYNTHETIC): median 95 %-circle coverage during outages 0-12 % (target ~95 %); p90 endpoint error
  250-400 m without SpeedNet, 66-328 m with it. Fixture-like drives behave far better (1-8 m), so random dynamics
  (stops, hard turns, speed changes, wrong-but-confident alignment) expose weaknesses.
- Not yet tried: inflating process noise for the accel/heading leak terms; a covariance floor tied to alignment confidence;
  inspecting the worst seeds (stops, alignment yaw error at confident=True, ZUPT thresholds).
- Plan: diagnose the worst seeds and calibrate covariance as part of P5/P6 (integrity + eval), since coverage is a headline metric.
