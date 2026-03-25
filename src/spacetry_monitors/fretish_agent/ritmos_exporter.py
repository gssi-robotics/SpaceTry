"""
ritmos_exporter.py
──────────────────
Exports artifacts in the exact formats consumed by RiTMOS and FRET.

RiTMOS expects two input artifacts:

1. A ``.fcs`` file — **JSON** with a ``robotSpec`` wrapper containing
   requirements (each with ``name``, ``fretish``, ``ptLTL``) and
   variable declarations.

2. A ``signals.yaml`` — flat signal map used by the RiTMOS ROS wrapper
   to subscribe to topics and feed Copilot extern streams::

       signals:
         - {name: obstacle_too_close, type: bool, topic: /spacetry/obstacle_too_close}
       numeric_bools:
         - {name: speed_ok, expr: "rover_linear_velocity <= 1.5"}

Optionally, we also export a ``fretRequirementsVariables.json`` in the
full FRET format (requirements + variables with idType/dataType).

The format follows the actual RiTMOS conventions found in:
  deps/ritmos/inputs/sample.fcs
  deps/ritmos/inputs/signals.yaml
  deps/ritmos/examples/robot_box/specs/fretRequirementsVariables.json
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import List

import yaml

from .fretish_writer import Requirement, requirements_to_fcs, requirements_to_fret_json
from .variable_mapper import VariableMap


# ── RiTMOS signals.yaml format ───────────────────────────────────────────────

def _variable_map_to_ritmos_signals(vmap: VariableMap) -> dict:
    """
    Convert a VariableMap to the RiTMOS signals.yaml format:

    signals:
      - {name: ..., type: bool|float64|int32, topic: ...}
    numeric_bools:
      - {name: ..., expr: "..."}
    """
    signals = []
    numeric_bools = []

    for name, mv in vmap.all_mapped().items():
        if mv.kind == "internal":
            # Internal/derived signals become numeric_bools if they
            # have an expression, otherwise regular bool signals
            if mv.derivation_expr:
                numeric_bools.append({
                    "name": name,
                    "expr": mv.derivation_expr,
                })
            # Derived bools with no expression are still bool signals
            # that the monitor node will compute
            continue

        # Map our datatype to RiTMOS types
        ritmos_type = "bool"
        if mv.unit in ("meters", "m/s", "radians"):
            ritmos_type = "float64"
        elif mv.unit == "percent":
            ritmos_type = "float64"
        elif mv.unit == "seconds":
            ritmos_type = "float64"
        elif mv.datatype in ("float", "float64", "double"):
            ritmos_type = "float64"
        elif mv.datatype in ("int", "int32"):
            ritmos_type = "int32"

        topic = mv.ros_topic or f"/spacetry/{name}"

        signals.append({
            "name": name,
            "type": ritmos_type,
            "topic": topic,
        })

    doc = {"signals": signals}
    if numeric_bools:
        doc["numeric_bools"] = numeric_bools
    return doc


# ── FRET variable list ──────────────────────────────────────────────────────

DATATYPE_MAP = {
    "bool": "boolean",
    "float": "double",
    "float64": "double",
    "int": "integer",
    "int32": "integer",
    "meters": "double",
    "m/s": "double",
    "radians": "double",
    "percent": "double",
    "seconds": "double",
}

IDTYPE_MAP = {
    "input": "Input",
    "output": "Output",
    "internal": "Internal",
}


def _build_fret_variables(vmap: VariableMap) -> List[dict]:
    """
    Build variable list in FRET's fretRequirementsVariables.json format.
    """
    variables = []
    for name, mv in vmap.all_mapped().items():
        fret_datatype = DATATYPE_MAP.get(mv.datatype, "boolean")
        if mv.unit and mv.unit != "bool":
            fret_datatype = DATATYPE_MAP.get(mv.unit, fret_datatype)

        fret_idtype = IDTYPE_MAP.get(mv.kind, "Input")

        var = {
            "project": "SpaceTry",
            "component_name": "rover",
            "variable_name": name,
            "reqs": mv.used_by if hasattr(mv, "used_by") else [],
            "dataType": fret_datatype,
            "idType": fret_idtype,
            "moduleName": "",
            "description": mv.notes or "",
            "assignment": mv.derivation_expr or "",
            "completed": True,
        }
        variables.append(var)
    return variables


# ── Main export function ────────────────────────────────────────────────────

def export_ritmos_artifacts(
    reqs: List[Requirement],
    vmap: VariableMap,
    output_dir: Path,
    signals_filename: str = "signals.yaml",
    spec_filename: str = "spec.fcs",
    fret_json_filename: str = "fretRequirementsVariables.json",
) -> dict:
    """
    Write RiTMOS and FRET artifacts to ``output_dir``.

    Produces three files:
    - ``signals.yaml``  — RiTMOS signal map
    - ``spec.fcs``      — RiTMOS .fcs specification (JSON)
    - ``fretRequirementsVariables.json`` — full FRET export

    Returns a dict with paths and any warnings.
    """
    output_dir.mkdir(parents=True, exist_ok=True)

    signals_path = output_dir / signals_filename
    spec_path = output_dir / spec_filename
    fret_json_path = output_dir / fret_json_filename

    # ── signals.yaml (RiTMOS format) ─────────────────────────────
    signals_doc = _variable_map_to_ritmos_signals(vmap)
    signals_content = yaml.dump(
        signals_doc,
        default_flow_style=None,
        sort_keys=False,
    )
    signals_path.write_text(signals_content)

    # ── spec.fcs (RiTMOS JSON format) ────────────────────────────
    fcs_content = requirements_to_fcs(reqs)
    spec_path.write_text(fcs_content)

    # ── fretRequirementsVariables.json (FRET format) ─────────────
    fret_variables = _build_fret_variables(vmap)
    fret_json_content = requirements_to_fret_json(reqs, fret_variables)
    fret_json_path.write_text(fret_json_content)

    # ── Warnings ──────────────────────────────────────────────────
    warnings = []
    if vmap.has_unmapped():
        warnings.append(
            f"WARNING: {len(vmap.unmapped)} unmapped signal(s) — "
            "signals.yaml may be incomplete."
        )
    unverified_in_map = [
        mv.name for mv in vmap.all_mapped().values() if not mv.verified
    ]
    if unverified_in_map:
        warnings.append(
            f"WARNING: {len(unverified_in_map)} unverified signal(s): "
            f"{', '.join(unverified_in_map)}"
        )

    # Check that all requirements have ptLTL
    missing_ptltl = [r.req_id for r in reqs if not r.ptLTL]
    if missing_ptltl:
        warnings.append(
            f"WARNING: {len(missing_ptltl)} requirement(s) missing ptLTL "
            f"formulas: {', '.join(missing_ptltl)}. "
            "RiTMOS needs ptLTL to generate monitors. "
            "Run 'fretish-agent formalize' or use FRET CLI to compute them."
        )

    return {
        "signals_path": str(signals_path),
        "spec_path": str(spec_path),
        "fret_json_path": str(fret_json_path),
        "warnings": warnings,
    }
