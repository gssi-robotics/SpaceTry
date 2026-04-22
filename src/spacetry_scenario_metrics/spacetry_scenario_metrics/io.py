from __future__ import annotations

import json
from dataclasses import asdict, is_dataclass
from pathlib import Path
from typing import Any

from .schema import ScenarioMetricsBundle


def _json_ready(value: Any) -> Any:
    if is_dataclass(value):
        return _json_ready(asdict(value))
    if isinstance(value, dict):
        return {key: _json_ready(item) for key, item in value.items()}
    if isinstance(value, list):
        return [_json_ready(item) for item in value]
    if isinstance(value, tuple):
        return [_json_ready(item) for item in value]
    if isinstance(value, Path):
        return str(value)
    return value


def bundle_to_dict(bundle: ScenarioMetricsBundle) -> dict[str, Any]:
    return _json_ready(bundle)


def write_metrics_json(bundle: ScenarioMetricsBundle, output_path: str | Path) -> None:
    path = Path(output_path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(bundle_to_dict(bundle), indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
