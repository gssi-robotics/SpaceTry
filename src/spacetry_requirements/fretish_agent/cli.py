#!/usr/bin/env python3
"""
cli.py — FRETish Agent command-line interface.

Usage:
    python -m fretish_agent.cli <command> [options]

Commands:
    inventory       Print the signal inventory
    lint            Lint a requirements YAML file
    check-signals   Check signal availability for a requirements file
    realize         Run realizability checking (or placeholder pre-check)
    formalize       Convert FRETish text to ptLTL via FRET CLI
    export          Export RiTMOS artifacts (spec.fcs + signals.yaml)
    advise          Run full pipeline and print revision advice
    ritmos          Export artifacts and show RiTMOS invocation
    walk            Interactive step-by-step pipeline with human review
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import yaml

from . import __version__
from .fretish_writer import Requirement, requirements_to_fcs, requirements_to_yaml
from .linter import format_lint_report, lint_requirements
from .realizability_runner import (
    RealizabilityResult,
    check_realizability,
    formalize_fretish,
    get_realizability_invocation,
)
from .revision_advisor import advise_revisions, format_revision_report
from .ritmos_exporter import export_ritmos_artifacts
from .signal_checker import check_signals, format_signal_report
from .signal_registry import SignalRegistry
from .variable_mapper import build_variable_map


# ── Helpers ──────────────────────────────────────────────────────────────────


def _load_requirements(path: Path) -> list[Requirement]:
    """Load requirements from a YAML file."""
    with open(path) as f:
        data = yaml.safe_load(f)
    reqs = []
    for entry in data.get("requirements", []):
        reqs.append(
            Requirement(
                req_id=entry["req_id"],
                scope=entry.get("scope", "globally"),
                condition=entry.get("condition", ""),
                component=entry.get("component", "the rover"),
                timing=entry.get("timing", "immediately"),
                response=entry.get("response", ""),
                description=entry.get("description", ""),
                kind=entry.get("kind", "guarantee"),
                signals_used=entry.get("signals_used", []),
                rationale=entry.get("rationale", ""),
                ptLTL=entry.get("ptLTL", ""),
            )
        )
    return reqs


def _registry(args) -> SignalRegistry:
    if args.inventory:
        return SignalRegistry(Path(args.inventory))
    return SignalRegistry()


def _resolve_input_path(path_str: str) -> Path:
    """Resolve a user-provided path from cwd, package root, or repo root."""
    candidate = Path(path_str)
    if candidate.is_file():
        return candidate

    package_root = Path(__file__).resolve().parents[1]
    repo_root = Path(__file__).resolve().parents[3]
    for base in (package_root, repo_root):
        resolved = base / path_str
        if resolved.is_file():
            return resolved
    return candidate


# ── Commands ─────────────────────────────────────────────────────────────────


def cmd_inventory(args):
    reg = _registry(args)
    print(f"{reg}\n")
    print("Verified signals:")
    for s in reg.verified_signals():
        print(f"  {s.name:30s}  {s.kind:10s}  {s.ros_topic or '(derived)'}")
    print("\nUnverified / planned signals:")
    for s in reg.unverified_signals():
        print(f"  {s.name:30s}  {s.kind:10s}  NOT IMPLEMENTED")


def cmd_lint(args):
    reg = _registry(args)
    reqs = _load_requirements(_resolve_input_path(args.requirements))
    issues = lint_requirements(reqs, reg)
    print(format_lint_report(issues))
    sys.exit(1 if any(i.severity == "error" for i in issues) else 0)


def cmd_check_signals(args):
    reg = _registry(args)
    reqs = _load_requirements(_resolve_input_path(args.requirements))
    issues = check_signals(reqs, reg)
    print(format_signal_report(issues))
    sys.exit(1 if any(i.severity == "error" for i in issues) else 0)


def cmd_realize(args):
    reg = _registry(args)
    reqs = _load_requirements(_resolve_input_path(args.requirements))
    vmap = build_variable_map(reqs, reg)
    result = check_realizability(reqs, vmap)
    print(result.summary())
    print("\nRealizability invocation:")
    print(get_realizability_invocation())


def cmd_export(args):
    reg = _registry(args)
    reqs = _load_requirements(_resolve_input_path(args.requirements))
    vmap = build_variable_map(reqs, reg)
    out_dir = Path(args.output_dir)
    result = export_ritmos_artifacts(reqs, vmap, out_dir)
    print(f"Exported signals:  {result['signals_path']}")
    print(f"Exported spec:     {result['spec_path']}")
    for w in result.get("warnings", []):
        print(f"  {w}")


def cmd_advise(args):
    """Full pipeline: lint → signal check → realizability → revision advice."""
    reg = _registry(args)
    reqs = _load_requirements(_resolve_input_path(args.requirements))
    vmap = build_variable_map(reqs, reg)

    print("=" * 60)
    print("VARIABLE MAP")
    print("=" * 60)
    print(vmap.summary())

    print("\n" + "=" * 60)
    print("LINT REPORT")
    print("=" * 60)
    lint_issues = lint_requirements(reqs, reg)
    print(format_lint_report(lint_issues))

    print("\n" + "=" * 60)
    print("SIGNAL CHECK")
    print("=" * 60)
    sig_issues = check_signals(reqs, reg)
    print(format_signal_report(sig_issues))

    print("\n" + "=" * 60)
    print("REALIZABILITY")
    print("=" * 60)
    result = check_realizability(reqs, vmap)
    print(result.summary())

    print("\n" + "=" * 60)
    print("REVISION ADVICE")
    print("=" * 60)
    revisions = advise_revisions(reqs, vmap, lint_issues, sig_issues, result)
    print(format_revision_report(revisions))

    print("\n" + "=" * 60)
    print("FRETISH REQUIREMENTS")
    print("=" * 60)
    for r in reqs:
        print(f"  [{r.kind.upper():10s}] {r.req_id}")
        print(f"    {r.fretish_text()}")
        print(f"    → {r.description}")
        print()

    print("Realizability invocation:")
    print(get_realizability_invocation())


def cmd_walk(args):
    """
    Interactive walk-through: runs the pipeline step by step, pausing
    after each phase for human review.

    At each gate the human can:
      [y] proceed to next step
      [e] edit requirements.yaml and re-run this step
      [s] skip this step
      [a] abort the pipeline

    The requirements file is **reloaded** after every edit, so changes
    made in an external editor (VS Code, Codex, vim) take effect
    immediately.
    """
    reg = _registry(args)
    req_path = Path(args.requirements)
    out_dir = Path(args.output_dir)

    def _gate(step_name: str) -> str:
        """Prompt the human. Returns 'y', 'e', 's', or 'a'."""
        while True:
            try:
                answer = input(
                    f"\n{'─' * 60}\n"
                    f"  [{step_name}] "
                    f"proceed / edit & re-run / skip / abort? [y/e/s/a]: "
                ).strip().lower()
            except (EOFError, KeyboardInterrupt):
                print()
                return "a"
            if answer in ("y", "e", "s", "a", ""):
                return answer or "y"
            print("  Please enter y, e, s, or a.")

    # ── Step 0: Show requirements ────────────────────────────────
    step = "REQUIREMENTS REVIEW"
    while True:
        reqs = _load_requirements(req_path)
        print(f"\n{'=' * 60}")
        print(f"STEP 0 — {step}")
        print(f"{'=' * 60}")
        print(f"Loaded {len(reqs)} requirement(s) from {req_path}\n")
        for r in reqs:
            print(f"  [{r.kind.upper():10s}] {r.req_id}")
            print(f"    {r.fretish_text()}")
            if r.description:
                print(f"    → {r.description.strip()}")
            print()

        choice = _gate(step)
        if choice == "a":
            print("Aborted.")
            return
        if choice == "e":
            print(f"  Edit {req_path} in your editor, then press Enter...")
            try:
                input()
            except (EOFError, KeyboardInterrupt):
                print("\nAborted.")
                return
            continue  # reload and re-display
        break  # 'y' or 's'

    # ── Step 1: Lint ─────────────────────────────────────────────
    step = "LINT"
    while True:
        reqs = _load_requirements(req_path)
        print(f"\n{'=' * 60}")
        print(f"STEP 1 — {step}")
        print(f"{'=' * 60}")
        lint_issues = lint_requirements(reqs, reg)
        print(format_lint_report(lint_issues))
        errors = [i for i in lint_issues if i.severity == "error"]
        if errors:
            print(f"  *** {len(errors)} error(s) found. Fix before proceeding.")

        choice = _gate(step)
        if choice == "a":
            print("Aborted.")
            return
        if choice == "e":
            print(f"  Edit {req_path}, then press Enter...")
            try:
                input()
            except (EOFError, KeyboardInterrupt):
                print("\nAborted.")
                return
            continue
        break

    # ── Step 2: Signal check ─────────────────────────────────────
    step = "SIGNAL CHECK"
    while True:
        reqs = _load_requirements(req_path)
        print(f"\n{'=' * 60}")
        print(f"STEP 2 — {step}")
        print(f"{'=' * 60}")
        sig_issues = check_signals(reqs, reg)
        print(format_signal_report(sig_issues))

        choice = _gate(step)
        if choice == "a":
            print("Aborted.")
            return
        if choice == "e":
            print(f"  Edit {req_path}, then press Enter...")
            try:
                input()
            except (EOFError, KeyboardInterrupt):
                print("\nAborted.")
                return
            continue
        break

    # ── Step 3: Variable mapping ─────────────────────────────────
    step = "VARIABLE MAPPING"
    reqs = _load_requirements(req_path)
    vmap = build_variable_map(reqs, reg)
    print(f"\n{'=' * 60}")
    print(f"STEP 3 — {step}")
    print(f"{'=' * 60}")
    print(vmap.summary())
    if vmap.has_unmapped():
        print("\n  *** Unmapped signals found. These are not in the inventory.")

    choice = _gate(step)
    if choice == "a":
        print("Aborted.")
        return

    # ── Step 4: Realizability ────────────────────────────────────
    step = "REALIZABILITY"
    if choice != "s":
        while True:
            reqs = _load_requirements(req_path)
            vmap = build_variable_map(reqs, reg)
            print(f"\n{'=' * 60}")
            print(f"STEP 4 — {step}")
            print(f"{'=' * 60}")
            result = check_realizability(reqs, vmap)
            print(result.summary())
            print()
            print("Invocation reference:")
            print(get_realizability_invocation())

            choice = _gate(step)
            if choice == "a":
                print("Aborted.")
                return
            if choice == "e":
                print(f"  Edit {req_path}, then press Enter...")
                try:
                    input()
                except (EOFError, KeyboardInterrupt):
                    print("\nAborted.")
                    return
                continue
            break
    else:
        result = RealizabilityResult(realizable=None, message="Skipped by user")
        lint_issues = lint_requirements(reqs, reg)
        sig_issues = check_signals(reqs, reg)

    # ── Step 5: Revision advice ──────────────────────────────────
    step = "REVISION ADVICE"
    reqs = _load_requirements(req_path)
    vmap = build_variable_map(reqs, reg)
    lint_issues = lint_requirements(reqs, reg)
    sig_issues = check_signals(reqs, reg)
    print(f"\n{'=' * 60}")
    print(f"STEP 5 — {step}")
    print(f"{'=' * 60}")
    revisions = advise_revisions(reqs, vmap, lint_issues, sig_issues, result)
    print(format_revision_report(revisions))

    choice = _gate(step)
    if choice == "a":
        print("Aborted.")
        return
    if choice == "e":
        print(f"  Edit {req_path}, then press Enter to proceed to export...")
        try:
            input()
        except (EOFError, KeyboardInterrupt):
            print("\nAborted.")
            return

    # ── Step 6: Export ───────────────────────────────────────────
    step = "EXPORT"
    reqs = _load_requirements(req_path)
    vmap = build_variable_map(reqs, reg)
    print(f"\n{'=' * 60}")
    print(f"STEP 6 — {step}")
    print(f"{'=' * 60}")
    export_result = export_ritmos_artifacts(reqs, vmap, out_dir)
    print(f"  spec.fcs                       → {export_result['spec_path']}")
    print(f"  signals.yaml                   → {export_result['signals_path']}")
    print(f"  fretRequirementsVariables.json  → {export_result['fret_json_path']}")
    for w in export_result.get("warnings", []):
        print(f"  {w}")

    print(f"\nRiTMOS invocation:")
    print(f"  ritmos ros package {export_result['spec_path']} \\")
    print(f"    --pkg spacetry_monitor \\")
    print(f"    --outdir ros_ws/src \\")
    print(f"    --signals-map {export_result['signals_path']} \\")
    print(f"    --compile-copilot")

    print(f"\n{'=' * 60}")
    print("PIPELINE COMPLETE")
    print(f"{'=' * 60}")


def cmd_formalize(args):
    """Convert FRETish text or a requirements YAML file to ptLTL using FRET CLI."""
    candidate_path = _resolve_input_path(args.fretish)
    if candidate_path.is_file():
        reqs = _load_requirements(candidate_path)
        failures = 0
        for req in reqs:
            ptltl = formalize_fretish(req.fretish_text())
            if ptltl:
                print(f"{req.req_id}: {ptltl}")
            else:
                print(f"{req.req_id}: FORMALIZATION FAILED")
                failures += 1
        sys.exit(1 if failures else 0)

    ptltl = formalize_fretish(args.fretish)
    if ptltl:
        print(f"ptLTL: {ptltl}")
        return

    print("FRET CLI not available or formalization failed.")
    print("Install FRET and add fretcli to PATH.")
    sys.exit(1)


def cmd_ritmos(args):
    """Show RiTMOS invocation command for the exported artifacts."""
    reg = _registry(args)
    reqs = _load_requirements(_resolve_input_path(args.requirements))
    vmap = build_variable_map(reqs, reg)
    out_dir = Path(args.output_dir)

    # Export first
    result = export_ritmos_artifacts(reqs, vmap, out_dir)
    for w in result.get("warnings", []):
        print(f"WARNING: {w}")

    spec_path = result["spec_path"]
    signals_path = result["signals_path"]

    print("\nExported artifacts:")
    print(f"  Spec:    {spec_path}")
    print(f"  Signals: {signals_path}")
    print(f"  FRET:    {result['fret_json_path']}")

    print("\nRiTMOS monitor generation command:")
    print(f"  ritmos ros package {spec_path} \\")
    print(f"    --pkg spacetry_monitor \\")
    print(f"    --outdir ros_ws/src \\")
    print(f"    --signals-map {signals_path} \\")
    print(f"    --compile-copilot")

    print("\nRiTMOS spec validation:")
    print(f"  ritmos spec read {spec_path}")
    print(f"  ritmos spec analyze {spec_path} --signals-map {signals_path}")


# ── Main ─────────────────────────────────────────────────────────────────────


def main():
    parser = argparse.ArgumentParser(
        prog="fretish-agent",
        description="SpaceTry FRETish writing and verification agent",
    )
    parser.add_argument(
        "--version", action="version", version=f"%(prog)s {__version__}"
    )
    parser.add_argument(
        "--inventory",
        default=None,
        help="Path to signal_inventory.yaml (default: bundled)",
    )

    sub = parser.add_subparsers(dest="command")

    # inventory
    sub.add_parser("inventory", help="Print the signal inventory")

    # lint
    p_lint = sub.add_parser("lint", help="Lint a requirements YAML")
    p_lint.add_argument("requirements", help="Path to requirements.yaml")

    # check-signals
    p_sig = sub.add_parser("check-signals", help="Check signal availability")
    p_sig.add_argument("requirements", help="Path to requirements.yaml")

    # realize
    p_real = sub.add_parser("realize", help="Run realizability checking")
    p_real.add_argument("requirements", help="Path to requirements.yaml")

    # export
    p_exp = sub.add_parser("export", help="Export RiTMOS artifacts")
    p_exp.add_argument("requirements", help="Path to requirements.yaml")
    p_exp.add_argument(
        "-o", "--output-dir", default=".", help="Output directory"
    )

    # advise
    p_adv = sub.add_parser("advise", help="Full pipeline with revision advice")
    p_adv.add_argument("requirements", help="Path to requirements.yaml")

    # formalize
    p_form = sub.add_parser("formalize", help="Convert FRETish text or a requirements YAML file to ptLTL via FRET CLI")
    p_form.add_argument("fretish", help="FRETish sentence (in quotes) or path to requirements.yaml")

    # ritmos
    p_ritmos = sub.add_parser("ritmos", help="Export artifacts and show RiTMOS invocation")
    p_ritmos.add_argument("requirements", help="Path to requirements.yaml")
    p_ritmos.add_argument("-o", "--output-dir", default=".", help="Output directory")

    # walk
    p_walk = sub.add_parser("walk", help="Interactive step-by-step pipeline with human review")
    p_walk.add_argument("requirements", help="Path to requirements.yaml")
    p_walk.add_argument("-o", "--output-dir", default=".", help="Output directory")

    args = parser.parse_args()
    if not args.command:
        parser.print_help()
        sys.exit(1)

    dispatch = {
        "inventory": cmd_inventory,
        "lint": cmd_lint,
        "check-signals": cmd_check_signals,
        "realize": cmd_realize,
        "export": cmd_export,
        "advise": cmd_advise,
        "formalize": cmd_formalize,
        "ritmos": cmd_ritmos,
        "walk": cmd_walk,
    }
    dispatch[args.command](args)


if __name__ == "__main__":
    main()
