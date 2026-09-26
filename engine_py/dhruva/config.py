"""Load configs/engine.json (single source of engine parameters)."""
import json
from pathlib import Path

DEFAULT_PATH = Path(__file__).resolve().parents[2] / "configs" / "engine.json"


def load_config(path=None):
    return json.loads(Path(path or DEFAULT_PATH).read_text(encoding="utf-8"))
