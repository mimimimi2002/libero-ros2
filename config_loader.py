"""Load JSON configs from the repository config/ directory."""
from __future__ import annotations

import json
from pathlib import Path
from typing import Any

_REPO_ROOT = Path(__file__).resolve().parent
_CONFIG_DIR = _REPO_ROOT / "config"


def config_dir() -> Path:
    return _CONFIG_DIR


def load_config(name: str) -> dict[str, Any]:
    """Load config/<name>.json (name may include or omit .json)."""
    filename = name if name.endswith(".json") else f"{name}.json"
    path = _CONFIG_DIR / filename
    if not path.is_file():
        raise FileNotFoundError(f"Config not found: {path}")
    with path.open("r", encoding="utf-8") as f:
        data = json.load(f)
    if not isinstance(data, dict):
        raise ValueError(f"Config root must be a JSON object: {path}")
    return data


def load_env_config() -> dict[str, Any]:
    return load_config("env")


def load_perception_config() -> dict[str, Any]:
    return load_config("perception")


def load_control_config() -> dict[str, Any]:
    return load_config("control")
