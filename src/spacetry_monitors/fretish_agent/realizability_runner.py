"""
realizability_runner.py
───────────────────────
Interface to FRETish realizability checking.

**IMPORTANT — PLACEHOLDER:**
The actual FRET realizability tool (``fret-realizability`` or equivalent)
is not known to be installed in this environment.  This module isolates
the integration behind a well-defined interface so that:

  1. The rest of the agent can be developed and tested without the tool.
  2. When the tool becomes available, only this module needs to change.

What must be filled in:
  - ``FRET_REALIZABILITY_CMD``: the shell command to invoke.
  - The expected input format (currently assumes the ``.fcs`` file).
  - The expected output format (stdout JSON or exit code).

If you know the correct command, set the environment variable
``FRET_REALIZABILITY_CMD`` (e.g. ``fret-realizability --check``).
"""

from __future__ import annotations

import json
import os
import shutil
import subprocess
import tempfile
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional

from .fretish_writer import Requirement, requirements_to_fcs
from .variable_mapper import VariableMap, variable_map_to_signals_yaml


# ── Configuration ────────────────────────────────────────────────────────────

# PLACEHOLDER: set via environment or replace with actual command
FRET_REALIZABILITY_CMD = os.environ.get(
    "FRET_REALIZABILITY_CMD",
    "",  # empty means "not configured"
)


@dataclass
class RealizabilityResult:
    """Outcome of a realizability check."""

    realizable: Optional[bool]  # True / False / None if tool not available
    message: str
    diagnostics: List[str] = field(default_factory=list)
    raw_output: str = ""

    def summary(self) -> str:
        if self.realizable is None:
            return f"SKIPPED: {self.message}"
        status = "REALIZABLE" if self.realizable else "UNREALIZABLE"
        lines = [f"{status}: {self.message}"]
        for d in self.diagnostics:
            lines.append(f"  - {d}")
        return "\n".join(lines)


def check_realizability(
    reqs: List[Requirement],
    vmap: VariableMap,
    work_dir: Optional[Path] = None,
) -> RealizabilityResult:
    """
    Run realizability checking on a set of requirements.

    If the FRET tool is not configured, returns a result with
    ``realizable=None`` and instructions for the user.
    """

    if not FRET_REALIZABILITY_CMD:
        return _placeholder_check(reqs, vmap)

    # ── Write temporary files for the tool ────────────────────────
    if work_dir is None:
        work_dir = Path(tempfile.mkdtemp(prefix="fretish_"))

    fcs_path = work_dir / "roverSpec_full.fcs"
    sig_path = work_dir / "signals_full.yaml"

    fcs_path.write_text(requirements_to_fcs(reqs))
    sig_path.write_text(variable_map_to_signals_yaml(vmap))

    # ── Invoke the tool ───────────────────────────────────────────
    cmd = f"{FRET_REALIZABILITY_CMD} --spec {fcs_path} --signals {sig_path}"
    try:
        proc = subprocess.run(
            cmd,
            shell=True,
            capture_output=True,
            text=True,
            timeout=120,
        )
    except FileNotFoundError:
        return RealizabilityResult(
            realizable=None,
            message=f"Realizability tool not found: {FRET_REALIZABILITY_CMD}",
        )
    except subprocess.TimeoutExpired:
        return RealizabilityResult(
            realizable=None,
            message="Realizability check timed out after 120 s",
        )

    # ── Parse output (best-effort) ────────────────────────────────
    raw = proc.stdout + proc.stderr
    realizable = proc.returncode == 0
    diagnostics = [
        line.strip()
        for line in raw.splitlines()
        if line.strip() and not line.startswith("#")
    ]

    return RealizabilityResult(
        realizable=realizable,
        message="Realizability check completed",
        diagnostics=diagnostics,
        raw_output=raw,
    )


def _placeholder_check(
    reqs: List[Requirement],
    vmap: VariableMap,
) -> RealizabilityResult:
    """
    Structural pre-check when the FRET tool is not available.

    Performs lightweight sanity checks that can catch obvious
    unrealizability causes without the full tool:
      - Contradictory requirements (same signal, conflicting responses)
      - Missing output signals
      - Assumptions that constrain outputs
    """
    diagnostics: List[str] = []

    # Check: guarantees must reference at least one output
    guarantees = [r for r in reqs if r.kind == "guarantee"]
    for g in guarantees:
        has_output = any(
            vmap.outputs.get(s) is not None
            for s in (g.signals_used or [])
        )
        if not has_output:
            diagnostics.append(
                f"{g.req_id}: guarantee does not reference any output signal "
                f"in its signals_used list — may be unrealizable"
            )

    # Check: unmapped signals
    if vmap.has_unmapped():
        for name, req_ids in vmap.unmapped.items():
            diagnostics.append(
                f"Signal '{name}' is unmapped (used in {', '.join(req_ids)})"
            )

    # Check: conflicting immediate responses
    immediate_responses: Dict[str, List[str]] = {}
    for r in guarantees:
        if r.timing in ("immediately", "always"):
            key = r.condition or "(unconditional)"
            immediate_responses.setdefault(key, []).append(
                f"{r.req_id}: {r.response}"
            )
    for cond, resps in immediate_responses.items():
        if len(resps) > 1:
            diagnostics.append(
                f"Potential conflict under condition '{cond}': "
                + "; ".join(resps)
            )

    return RealizabilityResult(
        realizable=None,
        message=(
            "FRET realizability tool is not configured. "
            "Set FRET_REALIZABILITY_CMD environment variable to enable. "
            "Structural pre-check completed below."
        ),
        diagnostics=diagnostics,
    )


def get_realizability_invocation(
    fcs_path: str = "roverSpec_full.fcs",
    sig_path: str = "signals_full.yaml",
) -> str:
    """
    Return the command that *would* be run for realizability checking.

    Useful for documentation and for the user to run manually.
    """
    if FRET_REALIZABILITY_CMD:
        return f"{FRET_REALIZABILITY_CMD} --spec {fcs_path} --signals {sig_path}"
    return (
        "# PLACEHOLDER: FRET realizability command not configured.\n"
        "# When available, the invocation would be:\n"
        "#   fret-realizability --check --spec roverSpec_full.fcs "
        "--signals signals_full.yaml\n"
        "#\n"
        "# Set FRET_REALIZABILITY_CMD env var to the actual command.\n"
        "# Expected inputs:\n"
        f"#   {fcs_path}  — FRETish requirements\n"
        f"#   {sig_path}  — signal/variable mapping\n"
        "# Expected output:\n"
        "#   exit code 0 = realizable, non-zero = unrealizable\n"
        "#   stdout/stderr = diagnostic messages"
    )
