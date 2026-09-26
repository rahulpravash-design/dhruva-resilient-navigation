"""NavState: the one navigation-state record that every UI (web, Android) consumes. See docs/CONTRACT.md.

Built from an engine timeline row. Only what the engine really produces is filled in; anything else is None so a UI
shows NOT MEASURED instead of a guess.
"""
import math

from ..geo import psi_to_heading_deg

MODES = ("GNSS", "DR", "DEGRADED")
GNSS_STATUS = ("OK", "DEGRADED", "UNAVAILABLE", "VALIDATING", "REJECTED")
DATA_SOURCES = ("LIVE", "REPLAY", "MOCK")
DATA_TYPES = ("REAL", "SIMULATED")


def _num(x, nd):
    return None if x is None or not math.isfinite(x) else round(float(x), nd)


def gnss_status(mode, trust):
    """GNSS status shown next to the mode. VALIDATING = engine still in DR while the trust score climbs back."""
    if mode == "SPOOF_REJECTED":
        return "REJECTED"
    if mode == "GNSS":
        return "OK" if trust is not None and trust >= 0.5 else "DEGRADED"
    if mode == "REACQUIRE" or (mode == "DR" and trust is not None and trust > 0.01):
        return "VALIDATING"
    return "UNAVAILABLE"


def nav_state(row, data_source, data_type):
    """One NavState dict from a timeline row (t, lat, lon, speed, psi, cov95_m, mode, gnss_trust)."""
    engine_mode = row["mode"]
    trust = _num(row["gnss_trust"], 3)
    mode = "DR" if engine_mode in ("SPOOF_REJECTED", "REACQUIRE") else engine_mode
    return {
        "timestamp": _num(row["t"], 3),
        "latitude": _num(row["lat"], 7),
        "longitude": _num(row["lon"], 7),
        "speedMps": _num(row["speed"], 3),
        "headingDeg": _num(float(psi_to_heading_deg(row["psi"])), 2),
        "positionCov95M": _num(row["cov95_m"], 2),
        "mode": mode,
        "gnssTrust": trust,
        "gnssStatus": gnss_status(engine_mode, trust),
        "routeStatus": "NOT_AVAILABLE",
        "dataSource": data_source,
        "dataType": data_type,
    }


def nav_states(timeline, data_source="REPLAY", data_type="SIMULATED"):
    """NavState list for a whole timeline DataFrame."""
    return [nav_state(r, data_source, data_type) for r in timeline.to_dict("records")]
