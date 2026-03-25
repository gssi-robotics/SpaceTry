"""
variable_mapper.py
──────────────────
Maps the variables used in a set of FRETish requirements to their signal
inventory entries and categorises them for RiTMOS and FRET export.

Produces a structured mapping that feeds into:
- RiTMOS ``signals.yaml`` (flat signal list + numeric_bools)
- FRET variable list (with idType, dataType)
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Set

import yaml

from .fretish_writer import Requirement, extract_signal_names
from .signal_registry import Signal, SignalRegistry


@dataclass
class MappedVariable:
    """A variable referenced in at least one requirement, with its mapping."""

    name: str
    kind: str  # input | output | internal
    ros_topic: Optional[str] = None
    msg_type: Optional[str] = None
    field_path: Optional[str] = None
    unit: str = "bool"
    verified: bool = True
    notes: Optional[str] = None
    used_in: List[str] = field(default_factory=list)  # list of req_ids
    used_by: List[str] = field(default_factory=list)   # FRET req _ids

    # Derivation expression for internal/derived signals
    # Used by RiTMOS numeric_bools (e.g. "speed <= 1.5")
    derivation_expr: str = ""

    @property
    def datatype(self) -> str:
        """Infer RiTMOS-compatible datatype from unit."""
        if self.unit == "bool":
            return "bool"
        if self.unit in ("meters", "m/s", "radians", "percent", "seconds"):
            return "float64"
        return "bool"


@dataclass
class VariableMap:
    """Complete variable mapping for a requirement set."""

    inputs: Dict[str, MappedVariable] = field(default_factory=dict)
    outputs: Dict[str, MappedVariable] = field(default_factory=dict)
    internals: Dict[str, MappedVariable] = field(default_factory=dict)
    unmapped: Dict[str, List[str]] = field(default_factory=dict)

    def all_mapped(self) -> Dict[str, MappedVariable]:
        return {**self.inputs, **self.outputs, **self.internals}

    def has_unmapped(self) -> bool:
        return len(self.unmapped) > 0

    def summary(self) -> str:
        lines = [
            f"Variable map: {len(self.inputs)} inputs, "
            f"{len(self.outputs)} outputs, "
            f"{len(self.internals)} internal, "
            f"{len(self.unmapped)} unmapped",
        ]
        if self.unmapped:
            lines.append("Unmapped signals (NOT in inventory):")
            for name, req_ids in sorted(self.unmapped.items()):
                lines.append(f"  {name} — used in {', '.join(req_ids)}")
        return "\n".join(lines)


# ── Derivation expressions for known internal signals ────────────────────────
# These map internal signal names to the expressions RiTMOS can use.

KNOWN_DERIVATIONS = {
    "obstacle_too_close": "obstacle_min_range < 1.0",
    "rover_is_moving": "rover_linear_velocity > 0.01",
    "speed_ok": "rover_linear_velocity <= 1.5",
}


def build_variable_map(
    reqs: List[Requirement],
    registry: SignalRegistry,
) -> VariableMap:
    """
    Walk through all requirements, extract signal names, and map them.
    """
    vmap = VariableMap()

    for r in reqs:
        explicit = set(r.signals_used) if r.signals_used else set()
        extracted = extract_signal_names(r)
        all_signals = explicit | extracted

        for sig_name in all_signals:
            sig: Optional[Signal] = registry.get(sig_name)
            if sig is None:
                vmap.unmapped.setdefault(sig_name, []).append(r.req_id)
                continue

            mv = MappedVariable(
                name=sig.name,
                kind=sig.kind,
                ros_topic=sig.ros_topic,
                msg_type=sig.msg_type,
                field_path=sig.field_path,
                unit=sig.unit,
                verified=sig.verified,
                notes=sig.notes,
                used_in=[],
                derivation_expr=KNOWN_DERIVATIONS.get(sig.name, ""),
            )

            target = {
                "input": vmap.inputs,
                "output": vmap.outputs,
                "internal": vmap.internals,
            }.get(sig.kind, vmap.internals)

            if sig_name not in target:
                target[sig_name] = mv
            target[sig_name].used_in.append(r.req_id)

    return vmap
