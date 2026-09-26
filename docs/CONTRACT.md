# DHRUVA engine contract v1 (frozen at Hour 2)

Changing anything here needs team sign-off. Python (`engine_py/contract.py`) and the Kotlin port must both follow this file.

## Trace format
A trace is a set of equal-length columns, one entry per row, ordered by time.
Rows arrive at the IMU rate; the rate is not fixed by the contract (synthetic data uses 10 Hz).

## Frames and units
| Item | Definition |
|---|---|
| `t` | epoch seconds, float, strictly increasing |
| `ax ay az` | m/s², **includes gravity**, phone body frame |
| `gx gy gz` | rad/s, phone body frame, right-handed |
| `mx my mz` | µT, phone body frame; **NaN when no magnetometer** |
| `gnss_lat gnss_lon` | degrees, WGS84 |
| `gnss_acc` | metres, 1-sigma horizontal accuracy as reported by the receiver |
| `gnss_speed` | m/s |
| `gnss_bearing` | degrees clockwise from true north, [0, 360); NaN when unavailable (e.g. stationary) |
| `cn0_mean` | dB-Hz, mean over satellites used in the fix |
| `n_sats` | count of satellites used in the fix |
| `gnss_valid` | bool |
| local frame | ENU metres about a reference point (equirectangular; valid for trips of ~10 km) |

Body frame convention used by the synthetic generator and the baseline: phone flat, x forward, y left, z up.
Real phones are not aligned to the vehicle; mount alignment is the engine's job (Phase 2), not the contract's.

## GNSS validity
`gnss_valid = True` means **a fresh fix arrived on this row**. It is not "GNSS is healthy".
On rows with `gnss_valid = False`, `gnss_lat` and `gnss_lon` must be NaN so nothing can use a stale position by accident.
GNSS typically arrives at 1 Hz, so most rows have `gnss_valid = False` even without an outage.
The first row of a trace must carry a fix.

## INPUT columns
`t, ax, ay, az, gx, gy, gz, mx, my, mz, gnss_lat, gnss_lon, gnss_acc, gnss_speed, gnss_bearing, cn0_mean, n_sats, gnss_valid`

Required finite on every row: `t, ax, ay, az, gx, gy, gz`.
Required finite on valid rows: `gnss_lat, gnss_lon, gnss_acc (>0), gnss_speed (>=0), cn0_mean, n_sats (>=0)`.

## OUTPUT columns
`t, lat, lon, cov_ee, cov_en, cov_nn, speed, heading, mode, gnss_trust`

| Column | Definition |
|---|---|
| `lat lon` | degrees, estimated position |
| `cov_ee cov_en cov_nn` | m², 2x2 east/north position covariance (positive semi-definite) |
| `speed` | m/s, >= 0 |
| `heading` | degrees clockwise from north, [0, 360) |
| `mode` | `GNSS` (fresh GNSS is driving the estimate), `DR` (dead reckoning), `DEGRADED` (dead reckoning with low confidence) |
| `gnss_trust` | 0 to 1, integrity monitor's trust in current GNSS; 0 while no GNSS is being used |

Output has exactly one row per input row, same `t`.

## Metrics (`engine_py/metrics.py`)
- **endpoint error**: distance between estimate and truth at the last evaluated row (m).
- **max error**: largest such distance over the evaluated rows (m).
- **drift %**: endpoint error / truth path length over the evaluated rows x 100.
- **95% coverage**: fraction of rows where truth lies inside the 95% ellipse of the reported covariance (chi-square 2 dof, 5.991). A well-calibrated engine scores about 0.95.
- **re-entry jump**: at each non-GNSS to GNSS transition, distance between the new estimate and where the previous estimate would have moved to (previous position + speed x dt along heading) (m). Reported as the maximum.

Metrics can be restricted to a window (e.g. an injected outage) with a mask.

## Reporting rules
Phone-only and vehicle-grade results are never combined. Results from `engine_py/synth.py` are `SIMULATED` and are not evidence of real-world performance.
