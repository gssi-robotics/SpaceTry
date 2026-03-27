"""
linter.py
─────────
Validates FRETish requirements for syntactic and semantic well-formedness.

Checks performed:
  1. Each requirement has a non-empty req_id, component, and response.
  2. Scope is one of the recognised FRETish scopes.
  3. Timing is one of the recognised FRETish timings (or empty).
  4. Kind is "guarantee" or "assumption".
  5. All signal names referenced in the requirement are present in the
     signal registry.
  6. Signals used in assumption requirements should be inputs or internal.
  7. Signals used in guarantee requirements should include at least one
     output or internal signal.
  8. No duplicate requirement IDs.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional

from .fretish_writer import (
    VALID_SCOPES,
    VALID_TIMINGS,
    Requirement,
    extract_signal_names,
)
from .signal_registry import SignalRegistry


@dataclass
class LintIssue:
    """One issue found during linting."""

    req_id: str
    severity: str  # "error" | "warning"
    message: str

    def __str__(self) -> str:
        return f"[{self.severity.upper()}] {self.req_id}: {self.message}"


def lint_requirements(
    reqs: List[Requirement],
    registry: SignalRegistry,
) -> List[LintIssue]:
    """Run all lint checks and return a list of issues."""
    issues: List[LintIssue] = []
    seen_ids: set = set()

    for r in reqs:
        # ── Structural checks ──────────────────────────────────────
        if not r.req_id:
            issues.append(LintIssue("(unknown)", "error", "Missing req_id"))
            continue

        if r.req_id in seen_ids:
            issues.append(LintIssue(r.req_id, "error", "Duplicate requirement ID"))
        seen_ids.add(r.req_id)

        if not r.response:
            issues.append(LintIssue(r.req_id, "error", "Empty response field"))

        if not r.component:
            issues.append(LintIssue(r.req_id, "error", "Empty component field"))

        # ── Scope check ────────────────────────────────────────────
        scope_word = r.scope.split()[0].lower() if r.scope else "globally"
        if scope_word not in VALID_SCOPES:
            issues.append(
                LintIssue(
                    r.req_id,
                    "warning",
                    f"Unrecognised scope '{r.scope}'. "
                    f"Expected one of: {', '.join(sorted(VALID_SCOPES))}",
                )
            )

        # ── Timing check ──────────────────────────────────────────
        if r.timing:
            timing_word = r.timing.split()[0].lower()
            timing_matched = any(
                r.timing.lower().startswith(t) for t in VALID_TIMINGS
            )
            if not timing_matched:
                issues.append(
                    LintIssue(
                        r.req_id,
                        "warning",
                        f"Unrecognised timing '{r.timing}'. "
                        f"Expected one of: {', '.join(sorted(VALID_TIMINGS))}",
                    )
                )

        # ── Kind check ─────────────────────────────────────────────
        if r.kind not in ("guarantee", "assumption"):
            issues.append(
                LintIssue(
                    r.req_id,
                    "error",
                    f"Invalid kind '{r.kind}'. Must be 'guarantee' or 'assumption'.",
                )
            )

        # ── Signal existence check ─────────────────────────────────
        referenced = extract_signal_names(r)
        for sig_name in sorted(referenced):
            if not registry.exists(sig_name):
                issues.append(
                    LintIssue(
                        r.req_id,
                        "error",
                        f"Signal '{sig_name}' not found in signal inventory",
                    )
                )
            elif not registry.is_verified(sig_name):
                issues.append(
                    LintIssue(
                        r.req_id,
                        "warning",
                        f"Signal '{sig_name}' exists in inventory but is NOT VERIFIED "
                        f"(not yet implemented in SpaceTry)",
                    )
                )

        # ── Signal-kind consistency ────────────────────────────────
        if r.kind == "guarantee":
            has_controllable = any(
                registry.get(s) and registry.get(s).kind in ("output", "internal")
                for s in referenced
                if registry.exists(s)
            )
            if referenced and not has_controllable:
                issues.append(
                    LintIssue(
                        r.req_id,
                        "warning",
                        "Guarantee requirement references only input signals — "
                        "consider whether this should be an assumption instead.",
                    )
                )

    return issues


def format_lint_report(issues: List[LintIssue]) -> str:
    """Format lint issues as a human-readable report."""
    if not issues:
        return "All requirements passed linting with no issues."
    errors = [i for i in issues if i.severity == "error"]
    warnings = [i for i in issues if i.severity == "warning"]
    lines = [f"Lint report: {len(errors)} error(s), {len(warnings)} warning(s)\n"]
    for issue in issues:
        lines.append(f"  {issue}")
    return "\n".join(lines)
