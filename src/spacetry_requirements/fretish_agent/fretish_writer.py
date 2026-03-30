"""
fretish_writer.py
─────────────────
Structured representation and serialisation of FRETish requirements.

A FRETish requirement has the canonical form:

    [SCOPE] [CONDITION] [COMPONENT] shall [TIMING] [RESPONSE]

This module provides:

* A ``Requirement`` dataclass for structured representation,
* Serialisation to the ``.fcs`` JSON format consumed by RiTMOS,
* Serialisation to the FRET Requirements+Variables JSON format,
* Serialisation to a human-readable YAML format,
* Parsing of ``.fcs`` JSON back into ``Requirement`` objects.

Output formats follow the actual FRET and RiTMOS conventions:
- .fcs files are JSON with a ``robotSpec`` wrapper.
- Variables follow FRET's ``idType`` (Input/Output/Internal).
- Requirement IDs containing "assumption" are treated as environment
  assumptions by FRET's realizability checker.
"""

from __future__ import annotations

import json
import re
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Set

import yaml


# ── FRETish fields ───────────────────────────────────────────────────────────

VALID_SCOPES = {
    "globally", "before", "after", "in", "notin",
    "onlyBefore", "onlyAfter", "onlyIn",
}

VALID_TIMINGS = {
    "always", "never", "eventually", "immediately", "within",
    "for", "after", "until", "before", "at the next timepoint",
}


@dataclass
class Requirement:
    """One FRETish requirement."""

    req_id: str             # e.g. "REQ_OBSTACLE_STOP" or "ASM_ENV_CLEARANCE"
    scope: str = "globally"
    condition: str = ""     # e.g. "if obstacle_too_close"
    component: str = "the rover"
    timing: str = "immediately"
    response: str = ""      # e.g. "satisfy cmd_move_stop"
    description: str = ""
    kind: str = "guarantee"  # "guarantee" | "assumption"
    signals_used: List[str] = field(default_factory=list)
    rationale: str = ""

    # ptLTL formula — filled in by FRET formalize or manually
    ptLTL: str = ""

    def fretish_text(self) -> str:
        """Render the requirement in canonical FRETish English."""
        parts = []
        if self.scope and self.scope != "globally":
            parts.append(self.scope)
        if self.condition:
            parts.append(self.condition)
        parts.append(f"{self.component} shall")
        if self.timing:
            parts.append(self.timing)
        parts.append(self.response)
        return " ".join(parts)

    @property
    def fret_reqid(self) -> str:
        """
        Requirement ID in the form FRET expects.

        FRET convention: IDs containing 'assumption' are treated as
        environment assumptions during realizability checking.
        """
        if self.kind == "assumption" and "assumption" not in self.req_id.lower():
            return f"{self.req_id}_assumption"
        return self.req_id

    def to_fret_requirement_dict(self) -> dict:
        """
        Produce a requirement dict in the format used by FRET's
        fretRequirementsVariables.json export.
        """
        return {
            "reqid": self.fret_reqid,
            "parent_reqid": "",
            "project": "SpaceTry",
            "rationale": self.rationale or self.description,
            "comments": "",
            "fulltext": self.fretish_text(),
            "status": "",
            "semantics": {
                "type": "nasa",
                "scope": self._scope_semantics(),
                "condition": self._condition_semantics(),
                "timing": self.timing,
                "response": "satisfaction",
                "variables": self.signals_used,
                "component_name": self._component_name(),
                "ptExpanded": self.ptLTL if self.ptLTL else "",
                "pt": self.ptLTL if self.ptLTL else "",
            },
        }

    def to_fcs_requirement_dict(self) -> dict:
        """
        Produce a requirement dict in the format used by RiTMOS .fcs files.
        """
        d: dict = {
            "name": self.fret_reqid,
            "fretish": self.fretish_text(),
            "ptLTL": self.ptLTL if self.ptLTL else "",
            "CoCoSpecCode": "",
            "PCTL": "",
        }
        return d

    def to_dict(self) -> dict:
        """Internal YAML-friendly dict."""
        return {
            "req_id": self.req_id,
            "fretish": self.fretish_text(),
            "scope": self.scope,
            "condition": self.condition,
            "component": self.component,
            "timing": self.timing,
            "response": self.response,
            "description": self.description,
            "kind": self.kind,
            "signals_used": self.signals_used,
            "rationale": self.rationale,
            "ptLTL": self.ptLTL,
        }

    def _component_name(self) -> str:
        """Extract bare component name (strip 'the ')."""
        return self.component.replace("the ", "").strip()

    def _scope_semantics(self) -> dict:
        if self.scope == "globally" or not self.scope:
            return {"type": "null"}
        return {"type": self.scope, "exclusive": False, "required": False}

    def _condition_semantics(self) -> str:
        return "regular" if self.condition else "null"


# ── Serialisation: YAML ──────────────────────────────────────────────────────

def requirements_to_yaml(reqs: List[Requirement]) -> str:
    """Serialise a list of requirements to readable YAML."""
    return yaml.dump(
        {"requirements": [r.to_dict() for r in reqs]},
        default_flow_style=False,
        sort_keys=False,
        width=120,
    )


# ── Serialisation: .fcs JSON (RiTMOS format) ────────────────────────────────

def requirements_to_fcs(reqs: List[Requirement]) -> str:
    """
    Serialise requirements to ``.fcs`` JSON format consumed by RiTMOS.

    The format is::

        {
          "robotSpec": {
            "Internal_variables": [...],
            "Other_variables": [...],
            "Functions": [],
            "Requirements": [
              {"name": "...", "fretish": "...", "ptLTL": "...", ...},
              ...
            ]
          }
        }
    """
    fcs = {
        "robotSpec": {
            "Internal_variables": [],
            "Other_variables": [],
            "Functions": [],
            "Requirements": [r.to_fcs_requirement_dict() for r in reqs],
        }
    }
    return json.dumps(fcs, indent=2)


def requirements_to_fret_json(
    reqs: List[Requirement],
    variables: List[dict],
) -> str:
    """
    Serialise requirements + variables to FRET's
    fretRequirementsVariables.json format.

    This is the format FRET imports/exports and is also consumed by RiTMOS.
    """
    doc = {
        "requirements": [r.to_fret_requirement_dict() for r in reqs],
        "variables": variables,
    }
    return json.dumps(doc, indent=4)


# ── Parsing: .fcs JSON ──────────────────────────────────────────────────────

def parse_fcs(text: str) -> List[Requirement]:
    """
    Parse a ``.fcs`` file (JSON) back into Requirement objects.

    Supports both the RiTMOS robotSpec format and the FRET export format.
    """
    data = json.loads(text)

    # RiTMOS robotSpec format
    if "robotSpec" in data:
        raw_reqs = data["robotSpec"].get("Requirements", [])
    elif "requirements" in data:
        raw_reqs = data["requirements"]
    elif "Requirements" in data:
        raw_reqs = data["Requirements"]
    else:
        return []

    reqs: List[Requirement] = []
    for r in raw_reqs:
        req_id = r.get("reqid") or r.get("name") or r.get("req_id", "UNKNOWN")
        fulltext = r.get("fulltext") or r.get("fretish", "")
        ptltl = r.get("ptLTL") or r.get("ptltl", "")

        # Detect assumptions by FRET convention (ID contains 'assumption')
        kind = "assumption" if "assumption" in req_id.lower() else "guarantee"

        reqs.append(
            Requirement(
                req_id=req_id,
                kind=kind,
                response=fulltext,
                description=r.get("rationale", ""),
                ptLTL=ptltl,
            )
        )
    return reqs


# ── Signal extraction ────────────────────────────────────────────────────────

def extract_signal_names(req: Requirement) -> Set[str]:
    """Return all signal-like identifiers referenced in a requirement's fields."""
    text = " ".join([req.scope, req.condition, req.response])
    tokens = set(re.findall(r"\b([a-z][a-z0-9_]{2,})\b", text))
    keywords = {
        "globally", "before", "after", "shall", "always", "never",
        "eventually", "immediately", "within", "the", "rover", "satisfy",
        "not", "and", "implies", "true", "false", "seconds", "for",
        "until", "notin", "onlybefore", "onlyafter", "onlyin",
        "environment", "then",
    }
    return tokens - keywords
