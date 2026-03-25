"""
signal_registry.py
──────────────────
Loads and queries the canonical SpaceTry signal inventory.

The inventory lives in ``config/signal_inventory.yaml`` and is the single
source of truth for which signals exist, their kinds (input / output /
internal), their ROS mappings, and whether they are actually verified
in the current codebase.
"""

from __future__ import annotations

import os
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional

import yaml


@dataclass
class Signal:
    """One entry in the signal inventory."""

    name: str
    kind: str  # "input" | "output" | "internal"
    ros_topic: Optional[str] = None
    msg_type: Optional[str] = None
    field_path: Optional[str] = None
    unit: str = ""
    verified: bool = False
    notes: str = ""


_DEFAULT_INVENTORY = Path(__file__).resolve().parent.parent / "config" / "signal_inventory.yaml"


class SignalRegistry:
    """Read-only registry of known SpaceTry signals."""

    def __init__(self, inventory_path: Optional[Path] = None):
        self._path = inventory_path or _DEFAULT_INVENTORY
        self._signals: Dict[str, Signal] = {}
        self._load()

    # ── public API ───────────────────────────────────────────────────

    def all_signals(self) -> Dict[str, Signal]:
        return dict(self._signals)

    def get(self, name: str) -> Optional[Signal]:
        return self._signals.get(name)

    def exists(self, name: str) -> bool:
        return name in self._signals

    def is_verified(self, name: str) -> bool:
        sig = self._signals.get(name)
        return sig is not None and sig.verified

    def inputs(self) -> List[Signal]:
        return [s for s in self._signals.values() if s.kind == "input"]

    def outputs(self) -> List[Signal]:
        return [s for s in self._signals.values() if s.kind == "output"]

    def internals(self) -> List[Signal]:
        return [s for s in self._signals.values() if s.kind == "internal"]

    def verified_signals(self) -> List[Signal]:
        return [s for s in self._signals.values() if s.verified]

    def unverified_signals(self) -> List[Signal]:
        return [s for s in self._signals.values() if not s.verified]

    def names(self) -> List[str]:
        return list(self._signals.keys())

    # ── loading ──────────────────────────────────────────────────────

    def _load(self):
        with open(self._path, "r") as fh:
            raw = yaml.safe_load(fh) or {}
        for name, attrs in raw.items():
            if not isinstance(attrs, dict):
                continue
            self._signals[name] = Signal(
                name=name,
                kind=attrs.get("kind", "internal"),
                ros_topic=attrs.get("ros_topic"),
                msg_type=attrs.get("msg_type"),
                field_path=attrs.get("field"),
                unit=attrs.get("unit", ""),
                verified=bool(attrs.get("verified", False)),
                notes=str(attrs.get("notes", "")),
            )

    def __repr__(self) -> str:
        v = sum(1 for s in self._signals.values() if s.verified)
        return f"<SignalRegistry {len(self._signals)} signals ({v} verified)>"
