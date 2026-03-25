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
    export          Export RiTMOS artifacts (signals_full.yaml + roverSpec_full.fcs)
    run-example     Run a built-in example scenario end-to-end
    advise          Run full pipeline and print revision advice
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
            )
        )
    return reqs


def _registry(args) -> SignalRegistry:
    if args.inventory:
        return SignalRegistry(Path(args.inventory))
    return SignalRegistry()


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
    reqs = _load_requirements(Path(args.requirements))
    issues = lint_requirements(reqs, reg)
    print(format_lint_report(issues))
    sys.exit(1 if any(i.severity == "error" for i in issues) else 0)


def cmd_check_signals(args):
    reg = _registry(args)
    reqs = _load_requirements(Path(args.requirements))
    issues = check_signals(reqs, reg)
    print(format_signal_report(issues))
    sys.exit(1 if any(i.severity == "error" for i in issues) else 0)


def cmd_realize(args):
    reg = _registry(args)
    reqs = _load_requirements(Path(args.requirements))
    vmap = build_variable_map(reqs, reg)
    result = check_realizability(reqs, vmap)
    print(result.summary())
    print("\nRealizability invocation:")
    print(get_realizability_invocation())


def cmd_export(args):
    reg = _registry(args)
    reqs = _load_requirements(Path(args.requirements))
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
    reqs = _load_requirements(Path(args.requirements))
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
    }
    dispatch[args.command](args)


if __name__ == "__main__":
    main()
