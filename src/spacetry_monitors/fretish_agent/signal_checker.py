"""
signal_checker.py
─────────────────
Verifies that every signal referenced in a set of FRETish requirements
is present in the SpaceTry signal inventory and flags issues.

Returns a structured report suitable for user display and for the
revision advisor.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Set

from .fretish_writer import Requirement, extract_signal_names
from .signal_registry import SignalRegistry


@dataclass
class SignalIssue:
    """One signal-related issue."""

    signal_name: str
    req_ids: List[str]
    severity: str  # "error" | "warning"
    message: str

    def __str__(self) -> str:
        reqs = ", ".join(self.req_ids)
        return f"[{self.severity.upper()}] {self.signal_name} (in {reqs}): {self.message}"


def check_signals(
    reqs: List[Requirement],
    registry: SignalRegistry,
) -> List[SignalIssue]:
    """
    Check all signals referenced across requirements.

    Returns:
      - error for signals not in inventory at all
      - warning for signals in inventory but not verified (not implemented)
    """
    # Collect which signals are referenced and by which requirements
    usage: Dict[str, List[str]] = {}
    for r in reqs:
        explicit = set(r.signals_used) if r.signals_used else set()
        extracted = extract_signal_names(r)
        for sig in explicit | extracted:
            usage.setdefault(sig, []).append(r.req_id)

    issues: List[SignalIssue] = []

    for sig_name, req_ids in sorted(usage.items()):
        sig = registry.get(sig_name)
        if sig is None:
            issues.append(
                SignalIssue(
                    signal_name=sig_name,
                    req_ids=req_ids,
                    severity="error",
                    message=(
                        f"Signal '{sig_name}' is NOT in the signal inventory. "
                        f"Either add it to signal_inventory.yaml or remove it from "
                        f"the requirements."
                    ),
                )
            )
        elif not sig.verified:
            issues.append(
                SignalIssue(
                    signal_name=sig_name,
                    req_ids=req_ids,
                    severity="warning",
                    message=(
                        f"Signal '{sig_name}' is in the inventory but NOT VERIFIED — "
                        f"it has no implementation in the current SpaceTry codebase. "
                        f"Notes: {sig.notes.strip()}"
                    ),
                )
            )

    return issues


def format_signal_report(issues: List[SignalIssue]) -> str:
    """Human-readable report."""
    if not issues:
        return "All signals verified: every referenced signal exists and is implemented."
    errors = [i for i in issues if i.severity == "error"]
    warnings = [i for i in issues if i.severity == "warning"]
    lines = [
        f"Signal check: {len(errors)} missing signal(s), "
        f"{len(warnings)} unverified signal(s)\n"
    ]
    for issue in issues:
        lines.append(f"  {issue}")
    return "\n".join(lines)
