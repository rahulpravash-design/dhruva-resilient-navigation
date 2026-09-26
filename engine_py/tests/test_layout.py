import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def test_package_layout():
    for sub in ("io", "sim", "align", "ekf", "speednet", "integrity", "mapmatch", "route", "metrics", "eval", "export"):
        assert (ROOT / "engine_py" / "dhruva" / sub / "__init__.py").is_file(), sub


def test_engine_config_has_spec_values():
    cfg = json.loads((ROOT / "configs" / "engine.json").read_text())
    assert cfg["ekf"]["nhc_sigma_mps"] == 0.2
    assert cfg["integrity"]["chi2_99_2dof"] == 9.21
    assert cfg["ps_benchmark"] == {"drift_pct_max": 10.0, "err_per_km_m_max": 100.0}
