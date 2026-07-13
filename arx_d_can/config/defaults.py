"""Hardware config paths."""

from __future__ import annotations

from pathlib import Path

CONFIG_DIR = Path(__file__).resolve().parent
DEFAULT_HW_CFG = CONFIG_DIR / "arx_d_can_dm.yaml"


def resolve_hw_cfg_path(hw_yaml: str | Path | None = None) -> Path:
    if hw_yaml is None:
        if not DEFAULT_HW_CFG.exists():
            raise FileNotFoundError(f"{DEFAULT_HW_CFG} not found")
        return DEFAULT_HW_CFG

    path = Path(hw_yaml)
    if path.is_absolute():
        if not path.exists():
            raise FileNotFoundError(f"hardware config not found: {path}")
        return path

    resolved = CONFIG_DIR / path
    if resolved.exists():
        return resolved
    raise FileNotFoundError(f"hardware config not found: {resolved}")
