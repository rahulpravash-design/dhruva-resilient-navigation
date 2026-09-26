"""NavState export validates against contracts/nav_state.schema.json and follows the mode/status rules."""
import json
from pathlib import Path

import jsonschema
import pandas as pd

from dhruva.export import gnss_status, nav_state, nav_states

SCHEMA = json.loads((Path(__file__).resolve().parents[2] / "contracts" / "nav_state.schema.json").read_text())


def _row(mode="GNSS", trust=1.0, cov=4.0):
    return {"t": 12.5, "lat": 28.6, "lon": 77.2, "speed": 14.0, "psi": 0.0, "cov95_m": cov, "mode": mode,
            "gnss_trust": trust}


def test_every_engine_mode_gives_a_valid_navstate():
    for mode, trust in (("GNSS", 1.0), ("DR", 0.0), ("DR", 0.4), ("DEGRADED", 0.0), ("REACQUIRE", 0.5),
                        ("SPOOF_REJECTED", 0.0)):
        jsonschema.validate(nav_state(_row(mode, trust), "REPLAY", "SIMULATED"), SCHEMA)


def test_heading_convention_and_mode_mapping():
    s = nav_state(_row("SPOOF_REJECTED", 0.0), "REPLAY", "SIMULATED")
    assert s["headingDeg"] == 90.0          # psi = 0 (east) is heading 90 deg
    assert s["mode"] == "DR" and s["gnssStatus"] == "REJECTED"
    assert s["routeStatus"] == "NOT_AVAILABLE"


def test_gnss_status_rules():
    assert gnss_status("GNSS", 1.0) == "OK"
    assert gnss_status("GNSS", 0.2) == "DEGRADED"
    assert gnss_status("DR", 0.0) == "UNAVAILABLE"
    assert gnss_status("DR", 0.3) == "VALIDATING"


def test_unmeasured_values_become_null_not_guesses():
    s = nav_state(_row(cov=float("nan"), trust=float("nan")), "REPLAY", "SIMULATED")
    assert s["positionCov95M"] is None and s["gnssTrust"] is None
    jsonschema.validate(s, SCHEMA)


def test_nav_states_over_a_timeline():
    tl = pd.DataFrame([_row(), _row("DR", 0.0)])
    out = nav_states(tl, "REPLAY", "SIMULATED")
    assert [o["mode"] for o in out] == ["GNSS", "DR"]
