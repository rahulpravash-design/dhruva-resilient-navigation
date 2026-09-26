"""Guards for the offline demo (web/): data is labelled SIMULATED, is self-consistent, and makes no banned claims."""
import hashlib
import json
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
BANNED = ("0.84%", "0.84 %", "NovAtel", "ISO 26262", "Secure Enclave", "$4.2B", "28 NavIC", "Pixel 7", "certified")
ARRAYS = ("t", "e", "n", "truth_e", "truth_n", "base_e", "base_n", "speed_kmh", "heading", "cov95_m", "trust", "mode")


def _demo():
    text = (ROOT / "web" / "data" / "demo.js").read_text(encoding="utf-8")
    return json.loads(re.sub(r"^window\.DHRUVA_DEMO\s*=\s*|;\s*$", "", text.strip()))


def test_demo_is_labelled_simulated_and_consistent():
    d = _demo()
    assert d["label"] == "SIMULATED"
    assert {s["id"] for s in d["scenarios"]} == {"outage", "spoof_step", "spoof_ramp"}
    for s in d["scenarios"]:
        assert s["label"] == "SIMULATED"
        assert len({len(s[k]) for k in ARRAYS}) == 1
        assert all(0 <= m < len(s["modes"]) for m in s["mode"])


def test_outage_scenario_enters_and_leaves_dead_reckoning():
    s = next(x for x in _demo()["scenarios"] if x["id"] == "outage")
    modes = [s["modes"][m] for m in s["mode"]]
    assert modes[0] == "GNSS" and "DR" in modes and modes[-1] == "GNSS"


def test_batch_data_traces_to_the_committed_run():
    text = (ROOT / "web" / "data" / "batch.js").read_text(encoding="utf-8")
    batch = json.loads(re.sub(r"^window\.DHRUVA_BATCH\s*=\s*|;\s*$", "", text.strip()))
    cases = (ROOT / "runs" / batch["run_id"] / "cases.jsonl").read_bytes()
    assert batch["label"] == "SYNTHETIC"
    assert hashlib.sha256(cases).hexdigest() == batch["cases_sha256"]
    assert len(cases.splitlines()) == batch["cases"]
    assert batch["names"].keys() == set(batch["ablations"])


def test_web_and_readme_have_no_banned_claims_or_external_urls():
    html = (ROOT / "web" / "index.html").read_text(encoding="utf-8")
    readme = (ROOT / "README.md").read_text(encoding="utf-8")
    for text in (html, readme):
        assert not [b for b in BANNED if b in text]
    assert not re.findall(r"(?:src|href)=[\"']https?://", html)
    for heading in ("# DHRUVA", "## Problem", "## Solution", "## Demo", "## Demo Scenario", "## Current Status",
                    "## Important limitation", "## Team"):
        assert heading in readme
