"""
variable_mapper.py
──────────────────
Maps the variables used in a set of FRETish requirements to their signal
inventory entries and categorises them for RiTMOS export.

Produces a structured mapping that can be exported to ``signals_full.yaml``.
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
    ros_topic: Optional[str]
    msg_type: Optional[str]
    field_path: Optional[str]
    unit: str
    verified: bool
    used_in: List[str] = field(default_factory=list)  # list of req_ids


@dataclass
class VariableMap:
    """Complete variable mapping for a requirement set."""

    inputs: Dict[str, MappedVariable] = field(default_factory=dict)
    outputs: Dict[str, MappedVariable] = field(default_factory=dict)
    internals: Dict[str, MappedVariable] = field(default_factory=dict)
    unmapped: Dict[str, List[str]] = field(default_factory=dict)  # name → [req_ids]

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


def build_variable_map(
    reqs: List[Requirement],
    registry: SignalRegistry,
) -> VariableMap:
    """
    Walk through all requirements, extract signal names, and map them.
    """
    vmap = VariableMap()

    for r in reqs:
        # Combine explicit signals_used list and auto-extracted names
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
                used_in=[],
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


def variable_map_to_signals_yaml(vmap: VariableMap) -> str:
    """
    Export the variable map in the ``signals_full.yaml`` format expected
    by RiTMOS.

    Schema (per variable)::

        variable_name:
          kind: input | output | internal
          ros_topic: /topic/name
          msg_type: pkg/msg/Type
          field: field.path
          datatype: bool | float | int
    """
    data: dict = {}
    for mv in sorted(vmap.all_mapped().values(), key=lambda v: v.name):
        entry: dict = {"kind": mv.kind}
        if mv.ros_topic:
            entry["ros_topic"] = mv.ros_topic
        if mv.msg_type:
            entry["msg_type"] = mv.msg_type
        if mv.field_path:
            entry["field"] = mv.field_path
        # Infer datatype from unit
        if mv.unit == "bool":
            entry["datatype"] = "bool"
        elif mv.unit in ("meters", "m/s", "radians", "percent", "seconds"):
            entry["datatype"] = "float"
        else:
            entry["datatype"] = "float"  # default
        data[mv.name] = entry

    return yaml.dump(data, default_flow_style=False, sort_keys=False, width=120)
