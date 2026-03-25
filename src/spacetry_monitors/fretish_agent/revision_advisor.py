"""
revision_advisor.py
───────────────────
Diagnoses realizability failures and suggests minimally invasive revisions.

Given a set of requirements, a variable map, lint issues, signal issues,
and a realizability result, this module produces human-readable advice on
what to change and why.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional

from .fretish_writer import Requirement
from .linter import LintIssue
from .realizability_runner import RealizabilityResult
from .signal_checker import SignalIssue
from .variable_mapper import VariableMap


@dataclass
class Revision:
    """One suggested revision."""

    req_id: str
    category: str  # "signal" | "conflict" | "assumption" | "timing" | "scope"
    suggestion: str
    rationale: str

    def __str__(self) -> str:
        return (
            f"[{self.category.upper()}] {self.req_id}:\n"
            f"  Suggestion: {self.suggestion}\n"
            f"  Rationale:  {self.rationale}"
        )


def advise_revisions(
    reqs: List[Requirement],
    vmap: VariableMap,
    lint_issues: List[LintIssue],
    signal_issues: List[SignalIssue],
    realizability: RealizabilityResult,
) -> List[Revision]:
    """Produce a list of suggested revisions ordered by priority."""
    revisions: List[Revision] = []

    # ── 1. Fix missing signals first (hard blockers) ─────────────
    missing_signals = [si for si in signal_issues if si.severity == "error"]
    for si in missing_signals:
        for req_id in si.req_ids:
            revisions.append(
                Revision(
                    req_id=req_id,
                    category="signal",
                    suggestion=(
                        f"Remove or replace signal '{si.signal_name}' — "
                        f"it does not exist in the signal inventory."
                    ),
                    rationale=(
                        f"The signal '{si.signal_name}' has no ROS topic or "
                        f"derivation path in the current SpaceTry system. "
                        f"Either add it to signal_inventory.yaml with a "
                        f"verified=true entry, or reformulate the requirement "
                        f"using only verified signals."
                    ),
                )
            )

    # ── 2. Flag unverified signals (soft blockers) ───────────────
    unverified = [si for si in signal_issues if si.severity == "warning"]
    for si in unverified:
        for req_id in si.req_ids:
            revisions.append(
                Revision(
                    req_id=req_id,
                    category="signal",
                    suggestion=(
                        f"Signal '{si.signal_name}' is planned but not implemented. "
                        f"Mark the requirement as PENDING or add an environment "
                        f"assumption that this signal will be provided."
                    ),
                    rationale=si.message,
                )
            )

    # ── 3. Fix lint errors ────────────────────────────────────────
    for li in lint_issues:
        if li.severity == "error":
            revisions.append(
                Revision(
                    req_id=li.req_id,
                    category="scope" if "scope" in li.message.lower() else "timing",
                    suggestion=f"Fix: {li.message}",
                    rationale="Lint error that prevents valid FRETish export.",
                )
            )

    # ── 4. Address guarantee/assumption misclassification ─────────
    for li in lint_issues:
        if "should be an assumption" in li.message.lower():
            revisions.append(
                Revision(
                    req_id=li.req_id,
                    category="assumption",
                    suggestion=(
                        "Reclassify this requirement as an assumption rather "
                        "than a guarantee — it only references input signals."
                    ),
                    rationale=li.message,
                )
            )

    # ── 5. Address realizability diagnostics ──────────────────────
    if realizability.diagnostics:
        for diag in realizability.diagnostics:
            # Try to extract req_id from diagnostic
            req_id = "(general)"
            for r in reqs:
                if r.req_id in diag:
                    req_id = r.req_id
                    break
            revisions.append(
                Revision(
                    req_id=req_id,
                    category="conflict",
                    suggestion=f"Review: {diag}",
                    rationale=(
                        "Flagged by realizability pre-check. May indicate "
                        "conflicting or vacuous requirements."
                    ),
                )
            )

    return revisions


def format_revision_report(revisions: List[Revision]) -> str:
    """Human-readable revision advice."""
    if not revisions:
        return "No revisions needed — requirements appear consistent and realizable."
    lines = [f"Revision advisor: {len(revisions)} suggestion(s)\n"]
    for i, rev in enumerate(revisions, 1):
        lines.append(f"{i}. {rev}\n")
    return "\n".join(lines)
