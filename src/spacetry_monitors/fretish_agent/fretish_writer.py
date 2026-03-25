"""
fretish_writer.py
─────────────────
Structured representation and serialisation of FRETish requirements.

A FRETish requirement has the canonical form:

    [SCOPE] [CONDITION] [COMPONENT] shall [TIMING] [RESPONSE]

Where each field is optional except COMPONENT and RESPONSE.

This module does **not** synthesise requirements from natural language —
that is the job of the agent prompt (LLM). Instead it provides:

* A ``Requirement`` dataclass for structured representation,
* Serialisation to the ``.fcs`` format expected by RiTMOS,
* Serialisation to a human-readable YAML format,
* Parsing of the ``.fcs`` format back into ``Requirement`` objects.
"""

from __future__ import annotations

import re
import textwrap
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Set

import yaml


# ── FRETish fields ───────────────────────────────────────────────────────────

VALID_SCOPES = {
    "globally",
    "before",
    "after",
    "in",
    "notin",
    "onlyBefore",
    "onlyAfter",
    "onlyIn",
}

VALID_TIMINGS = {
    "always",
    "never",
    "eventually",
    "immediately",
    "within",
    "for",
    "after",
    "until",
    "before",
    "at the next timepoint",
}


@dataclass
class Requirement:
    """One FRETish requirement."""

    req_id: str  # e.g. "REQ_OBSTACLE_STOP"
    scope: str = "globally"  # FRETish scope
    condition: str = ""  # e.g. "if obstacle_too_close"
    component: str = "the rover"  # component under specification
    timing: str = "immediately"  # FRETish timing
    response: str = ""  # e.g. "satisfy cmd_move_stop"
    description: str = ""  # plain-English explanation
    kind: str = "guarantee"  # "guarantee" | "assumption"
    signals_used: List[str] = field(default_factory=list)

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

    def to_dict(self) -> dict:
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
        }


# ── Serialisation helpers ────────────────────────────────────────────────────


def requirements_to_yaml(reqs: List[Requirement]) -> str:
    """Serialise a list of requirements to readable YAML."""
    return yaml.dump(
        {"requirements": [r.to_dict() for r in reqs]},
        default_flow_style=False,
        sort_keys=False,
        width=120,
    )


def requirements_to_fcs(reqs: List[Requirement]) -> str:
    """
    Serialise requirements to ``.fcs`` (FRET Compact Specification) format.

    The ``.fcs`` format expected by RiTMOS is one requirement per block::

        // REQ_ID (kind) — description
        SCOPE CONDITION COMPONENT shall TIMING RESPONSE

    Assumptions and guarantees are distinguished by the ``kind`` tag.
    """
    lines: List[str] = []
    for r in reqs:
        lines.append(f"// {r.req_id} ({r.kind}) — {r.description}")
        lines.append(r.fretish_text())
        lines.append("")
    return "\n".join(lines)


def parse_fcs(text: str) -> List[Requirement]:
    """
    Parse a ``.fcs`` file back into Requirement objects.

    This is a best-effort parser.  It extracts the ``req_id``, ``kind``,
    ``description``, and raw FRETish text but does **not** decompose the
    FRETish text back into scope/condition/timing/response fields.
    """
    reqs: List[Requirement] = []
    lines = text.strip().splitlines()
    i = 0
    while i < len(lines):
        line = lines[i].strip()
        if line.startswith("//"):
            m = re.match(
                r"//\s*(\S+)\s*\((\w+)\)\s*[—–-]\s*(.*)", line
            )
            if m:
                req_id, kind, desc = m.group(1), m.group(2), m.group(3).strip()
                i += 1
                fretish_line = lines[i].strip() if i < len(lines) else ""
                reqs.append(
                    Requirement(
                        req_id=req_id,
                        kind=kind,
                        description=desc,
                        response=fretish_line,  # store raw line in response for now
                    )
                )
        i += 1
    return reqs


def extract_signal_names(req: Requirement) -> Set[str]:
    """Return all signal-like identifiers referenced in a requirement's fields."""
    text = " ".join(
        [req.scope, req.condition, req.response]
    )
    # Match snake_case identifiers (likely signal names)
    tokens = set(re.findall(r"\b([a-z][a-z0-9_]{2,})\b", text))
    # Remove FRETish keywords
    keywords = {
        "globally", "before", "after", "shall", "always", "never",
        "eventually", "immediately", "within", "the", "rover", "satisfy",
        "not", "and", "implies", "true", "false", "seconds", "for",
        "until", "notin", "onlybefore", "onlyafter", "onlyin",
    }
    return tokens - keywords
