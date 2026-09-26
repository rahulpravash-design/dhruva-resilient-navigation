# Limitations

Read this before quoting any DHRUVA number.

## Evidence
- Every result comes from a simulator with an ASSUMED phone IMU/GNSS model. There is no real drive, no public dataset and
  no on-device run. Simulated results show engine behaviour inside those assumptions; they are not evidence of real-world
  accuracy.
- SpeedNet (47,394 parameters) was trained on SYNTHETIC drives with an ASSUMED speed-dependent vibration model
  (`docs/SPEEDNET_ASSUMPTIONS.md`). It may not transfer to real phones and vehicles.
- The web demo shows ONE drive (seed 4000, first `test`-split drive, not chosen by result). It is a demonstration, not a statistic.
- The full evaluation matrix (`python -m dhruva.eval`) is implemented but a full run is not part of the committed results
  unless `docs/RESULTS.md` exists and says otherwise.
- Phone-grade and vehicle-grade results are never mixed. No vehicle-grade result exists.
- No map-matched number exists: the matcher and router are unit-tested on synthetic graphs and are not wired into the engine.

## Known engine issues (`docs/BLOCKERS.md`)
- B3: with an estimated mount the endpoint error is 2.75 m against 1.71 m with the true mount on the test setup, so the
  planned "within 10 % of perfect mount" gate is not met.
- B5: alignment quality is not represented in the filter; some drives keep 3 to 6 degrees of tilt and 5 to 11 degrees of yaw
  error while reporting confidence, and give the worst outage errors.
- B6: the displayed position can step 3 to 4 m in one tick just before GNSS re-lock after a long outage (target 2 m). A slow
  ramp spoof (2 m/s) takes about 15 to 19 s to be rejected, and a spoofer that starts during an outage cannot be told apart
  from a normal re-lock.
- Some outages have a heavy tail: maximum errors of several hundred metres were seen on some validation drives.

## Scope
- Spoofing: two controlled simulated attacks (500 m step, 2 m/s drift east). This is not universal spoofing detection.
- The integrity thresholds were tuned on the `val` split only. The `test` split was not used for tuning.
- Map: the web demo is a plain local trajectory view. There is no basemap, no roads and no offline tiles.
- The Android app and the Kotlin engine do not exist yet, so nothing runs on a phone.
- No claims of centimetre or millimetre accuracy, certification, RTK-class accuracy or guaranteed underground navigation.
