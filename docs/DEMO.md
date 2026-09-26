# Demo guide

## Start
```
python -m http.server 8765 --directory web
```
Open http://localhost:8765/index.html. Everything is offline. `web/data/demo.js` is committed; regenerate it from the real
engine with `python scripts/make_demo.py` (needs `pip install -r requirements.txt -r requirements-ml.txt`, about 1 minute).

The banner "DATA: SIMULATED REPLAY" is always visible: the drive is synthetic and is not a field measurement.

## Suggested run (about 2 minutes)
1. **Navigation HUD.** Point out DHRUVA, MODE, GNSS TRUST, SPEED, POSITION CONFIDENCE and DATA = SIMULATED.
2. Press **[ START DEMO ]** (plays the whole drive at 6x). The four-step strip shows GNSS MODE, then DEAD RECKONING
   (trust LOW), then GNSS RECOVERY (validating GNSS), then GNSS MODE.
3. Or press **[ SIMULATE GNSS OUTAGE ]** to start 12 s before the 60 s outage (4x).
4. Watch the amber ring: the filter's own 95 % radius grows during the outage and shrinks after GNSS returns.
5. **Replay / GNSS Outage.** Scrub the timeline. The chart compares the position error of DHRUVA against the simple gyro
   baseline (simulated ground truth). Untick "show baseline" to hide the orange track.
6. Pick **Spoof: 500 m step** or **Spoof: 2 m/s drift** to show a SIMULATED SPOOF SCENARIO: the monitor rejects the
   displaced fixes and DHRUVA carries on from IMU, speed and constraints.
7. **Technical Metrics.** Read the numbers for the selected scenario (n = 1, SIMULATED), then read the NOT MEASURED list.

## What to say
DHRUVA does not replace GNSS. It keeps a useful position estimate going when GNSS is unavailable, degraded or untrustworthy.
The numbers are from a simulator; real-drive validation and the Android app are still to do.

## What not to say
No accuracy claims beyond the on-screen simulated numbers, no "field tested", no "certified", no "detects all spoofing".
