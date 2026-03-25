"""
realizability_runner.py
───────────────────────
Invoke realizability checking via FRET's CLI or structural pre-check.

FRET CLI realizability invocation:

    fretcli realizability <project> <component> [--solver kind2|jkind]
        [--timeout N] [--diagnose] [--json [file]]

FRET CLI formalize invocation:

    fretcli formalize '<fretish sentence>' -l pt

Dependencies for realizability:
- Z3 SMT solver (required for all engines)
- Kind2 v2.2.0 (NOT v2.3.0) or JKind + JRealizability
"""

from __future__ import annotations

import json
import os
import shutil
import subprocess
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional

from .fretish_writer import Requirement
from .variable_mapper import VariableMap


@dataclass
class RealizabilityResult:
    """Result of a realizability check."""

    realizable: Optional[bool]  # True/False/None if tool unavailable
    message: str = ""
    diagnostics: List[str] = field(default_factory=list)
    conflicts: List[List[str]] = field(default_factory=list)
    raw_output: str = ""

    def summary(self) -> str:
        if self.realizable is None:
            return f"REALIZABILITY: SKIPPED — {self.message}"
        tag = "REALIZABLE" if self.realizable else "UNREALIZABLE"
        lines = [f"REALIZABILITY: {tag}"]
        if self.message:
            lines.append(f"  {self.message}")
        for d in self.diagnostics:
            lines.append(f"  - {d}")
        if self.conflicts:
            lines.append("  Conflicting requirement sets:")
            for i, c in enumerate(self.conflicts, 1):
                lines.append(f"    Conflict {i}: {', '.join(c)}")
        return "\n".join(lines)


# ── FRET CLI integration ────────────────────────────────────────────────────

def _find_fretcli() -> Optional[str]:
    """Locate the FRET CLI binary."""
    env_path = os.environ.get("FRET_CLI_PATH")
    if env_path and os.path.isfile(env_path):
        return env_path
    repo_root = Path(__file__).resolve().parents[3]
    candidates = [
        shutil.which("fretcli"),
        shutil.which("fret"),
        str(repo_root / ".spacetry" / "bin" / "fretcli"),
        str(repo_root / "deps" / "fret" / "tools" / "Scripts" / "cli" / "fretcli"),
    ]
    fret_dir = os.environ.get("FRET_HOME")
    if fret_dir:
        cli_path = os.path.join(fret_dir, "fret-electron", "app", "cli", "fretCLI.js")
        if os.path.isfile(cli_path):
            candidates.append(cli_path)
    for c in candidates:
        if c and os.path.isfile(c):
            return c
    return None


def formalize_fretish(fretish_text: str) -> Optional[str]:
    """
    Use FRET CLI to convert a FRETish sentence to past-time LTL (ptLTL).
    Returns the ptLTL formula string, or None if FRET CLI is unavailable.
    """
    fretcli = _find_fretcli()
    if not fretcli:
        return None
    def _extract_formula(output: str) -> Optional[str]:
        for line in output.splitlines():
            text = line.strip()
            if not text:
                continue
            if text.startswith(">") or text.startswith("npm "):
                continue
            if text.startswith("Error "):
                continue
            if text.startswith("Initialized project properties"):
                continue
            return text
        return None

    attempts = [fretish_text]
    normalized = fretish_text.replace(" satisfy not ", " satisfy ! ")
    if normalized != fretish_text:
        attempts.append(normalized)

    try:
        for attempt in attempts:
            result = subprocess.run(
                [fretcli, "formalize", attempt, "-l", "pt"],
                capture_output=True, text=True, timeout=30,
            )
            if result.returncode == 0:
                formula = _extract_formula(result.stdout)
                if formula:
                    return formula
    except (subprocess.TimeoutExpired, FileNotFoundError, OSError):
        pass
    return None


def check_realizability_fret(
    project: str, component: str,
    solver: str = "kind2", timeout: int = 900, diagnose: bool = True,
) -> RealizabilityResult:
    """
    Run FRET CLI realizability checking.

    Prerequisites: FRET installed, Z3 + Kind2/JKind on PATH,
    project + component with completed variable mapping in FRET's DB.
    """
    fretcli = _find_fretcli()
    if not fretcli:
        return RealizabilityResult(
            realizable=None,
            message="FRET CLI not found. Set FRET_CLI_PATH or add fretcli to PATH.",
        )
    cmd = [fretcli, "realizability", project, component,
           "--solver", solver, "--timeout", str(timeout), "--json"]
    if diagnose:
        cmd.append("--diagnose")
    try:
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout + 30)
    except subprocess.TimeoutExpired:
        return RealizabilityResult(realizable=None, message=f"FRET timed out after {timeout}s.")
    except FileNotFoundError:
        return RealizabilityResult(realizable=None, message="FRET CLI binary not found.")

    raw = result.stdout + result.stderr
    try:
        data = json.loads(result.stdout)
        return _parse_fret_json_result(data, raw)
    except (json.JSONDecodeError, KeyError):
        if "REALIZABLE" in raw and "UNREALIZABLE" not in raw:
            return RealizabilityResult(realizable=True, raw_output=raw)
        elif "UNREALIZABLE" in raw:
            return RealizabilityResult(realizable=False, raw_output=raw)
        return RealizabilityResult(realizable=None, message=f"Cannot parse FRET output (exit {result.returncode})", raw_output=raw)


def _parse_fret_json_result(data: dict, raw: str) -> RealizabilityResult:
    """Parse FRET's JSON realizability output."""
    components = data.get("systemComponents", [])
    if not components:
        return RealizabilityResult(realizable=None, message="No components in FRET output", raw_output=raw)
    comp = components[0]
    result_obj = comp.get("compositional") or comp.get("monolithic", {})
    result_str = result_obj.get("result", "UNKNOWN")
    realizable = {"REALIZABLE": True, "UNREALIZABLE": False}.get(result_str)
    diagnostics, conflicts = [], []
    diagnosis = result_obj.get("diagnosisReport", {})
    if diagnosis and diagnosis.get("Conflicts"):
        for conflict in diagnosis["Conflicts"]:
            conflict_reqs = conflict.get("Conflict", [])
            conflicts.append(conflict_reqs)
            diagnostics.append(f"Conflict: {', '.join(conflict_reqs)}")
    for cc in result_obj.get("connectedComponents", []):
        if cc.get("result") == "UNREALIZABLE":
            diagnostics.append(f"CC {cc.get('ccName', '?')} is UNREALIZABLE")
            cc_diag = cc.get("diagnosisReport", {})
            if cc_diag and cc_diag.get("Conflicts"):
                for c in cc_diag["Conflicts"]:
                    conflicts.append(c.get("Conflict", []))
    return RealizabilityResult(realizable=realizable, message=f"Result: {result_str}",
                               diagnostics=diagnostics, conflicts=conflicts, raw_output=raw)


# ── Structural pre-check ─────────────────────────────────────────────────────

def _structural_precheck(reqs: List[Requirement], vmap: VariableMap) -> List[str]:
    issues = []
    output_names = set(vmap.outputs.keys())
    guarantees = [r for r in reqs if r.kind == "guarantee"]
    for r in guarantees:
        req_signals = set(r.signals_used) if r.signals_used else set()
        if req_signals and not (req_signals & output_names):
            issues.append(f"{r.req_id}: guarantee references no output signals — may be unrealizable")
    if vmap.has_unmapped():
        for name, req_ids in vmap.unmapped.items():
            issues.append(f"'{name}' (used in {', '.join(req_ids)}) not in signal inventory")
    by_condition: Dict[str, List[Requirement]] = {}
    for r in guarantees:
        by_condition.setdefault(r.condition or "(unconditional)", []).append(r)
    for cond, group in by_condition.items():
        if len(group) > 1:
            issues.append(f"Potential conflict under '{cond}': " +
                          "; ".join(f"{r.req_id}: {r.response}" for r in group))
    return issues


# ── Main entry ──────────────────────────────────────────────────────────────

def check_realizability(
    reqs: List[Requirement], vmap: VariableMap,
    work_dir: Optional[Path] = None,
) -> RealizabilityResult:
    fretcli = _find_fretcli()
    if fretcli:
        result = check_realizability_fret("SpaceTry", "rover")
        if result.realizable is not None:
            return result
    issues = _structural_precheck(reqs, vmap)
    msg_suffix = "See docs/architecture.md for FRET + solver setup." if not fretcli else ""
    if issues:
        return RealizabilityResult(
            realizable=None,
            message=f"FRET not available. Structural pre-check found issues. {msg_suffix}",
            diagnostics=issues,
        )
    return RealizabilityResult(
        realizable=None,
        message=f"FRET not available. Structural pre-check passed. {msg_suffix}",
    )


def get_realizability_invocation(reqs: Optional[List[Requirement]] = None) -> str:
    fretcli = _find_fretcli()
    if fretcli:
        return (
            f"# FRET CLI at: {fretcli}\n"
            f"#   fretcli realizability SpaceTry rover --diagnose --json\n"
        )
    return (
        "# FRET CLI not found.\n"
        "#\n"
        "# Setup:\n"
        "#   cd deps/fret/fret-electron && npm install && npm run build-cli\n"
        "#   Install Z3 v4.14.1 + Kind2 v2.2.0 (NOT v2.3.0) on PATH\n"
        "#\n"
        "# For monitor synthesis without realizability:\n"
        "#   ritmos ros package spec.fcs --signals-map signals.yaml \\\n"
        "#     --pkg spacetry_monitor --outdir ros_ws/src --compile-copilot\n"
    )
