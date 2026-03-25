# FRETish Agent — Architecture

## Overview

The FRETish agent transforms natural-language mission/scenario descriptions
into minimal, verifiable safety requirements in FRETish form, suitable for
RiTMOS monitor synthesis.

## Module structure

```
src/spacetry_monitors/
├── fretish_agent/             # Python package
│   ├── __init__.py
│   ├── cli.py                 # Command-line interface (entry point)
│   ├── signal_registry.py     # Load & query signal inventory
│   ├── fretish_writer.py      # Requirement dataclass + serialisation
│   ├── linter.py              # Syntax & semantic validation
│   ├── variable_mapper.py     # Signal → variable mapping for RiTMOS
│   ├── signal_checker.py      # Verify signals exist in SpaceTry
│   ├── realizability_runner.py # FRET realizability (placeholder)
│   ├── revision_advisor.py    # Diagnose failures & suggest fixes
│   └── ritmos_exporter.py     # Export signals_full.yaml + roverSpec_full.fcs
├── config/
│   ├── signal_inventory.yaml  # Canonical signal ground truth
│   ├── fretish_templates.yaml # Common requirement patterns
│   └── agent_prompt.md        # System prompt for LLM-assisted mode
├── examples/
│   ├── scenario_rock_survey/  # Example 1: full end-to-end
│   └── scenario_safe_return/  # Example 2: with unverified signals
├── tests/                     # pytest test suite
├── docs/                      # This documentation
├── package.xml                # ROS 2 package manifest
├── CMakeLists.txt             # Build file
└── setup.py                   # Python package setup
```

## Data flow

```
Natural-language scenario
        │
        ▼
┌──────────────────┐
│  Agent / User    │  Writes requirements.yaml
│  (LLM or human)  │  using templates + signal inventory
└────────┬─────────┘
         │
         ▼
┌──────────────────┐     ┌──────────────────┐
│  signal_registry │◄────│ signal_inventory  │
│                  │     │     .yaml         │
└────────┬─────────┘     └──────────────────┘
         │
         ▼
┌──────────────────┐
│  linter          │  Validates syntax, signals, kinds
└────────┬─────────┘
         │
         ▼
┌──────────────────┐
│  signal_checker  │  Flags missing/unverified signals
└────────┬─────────┘
         │
         ▼
┌──────────────────┐
│  variable_mapper │  Builds I/O/internal mapping
└────────┬─────────┘
         │
         ▼
┌──────────────────┐
│  realizability   │  Invokes FRET tool (or placeholder)
│  _runner         │
└────────┬─────────┘
         │
         ▼
┌──────────────────┐
│ revision_advisor │  Suggests fixes if needed
└────────┬─────────┘
         │
         ▼
┌──────────────────┐
│ ritmos_exporter  │  Writes signals_full.yaml
│                  │  Writes roverSpec_full.fcs
└──────────────────┘
```

## User interaction loop

1. User provides a natural-language scenario description.
2. Agent (LLM or human) writes `requirements.yaml` using the signal
   inventory and templates as reference.
3. User runs `fretish-agent advise requirements.yaml` to get a full
   report: lint issues, signal availability, realizability status,
   and revision suggestions.
4. User revises requirements based on advice.
5. When satisfied, user runs `fretish-agent export requirements.yaml -o output/`
   to produce the RiTMOS artifacts.

## FRET realizability integration

The `realizability_runner.py` module is designed as a **pluggable interface**.
Currently, the actual FRET realizability tool command is not known. The
module provides:

- A placeholder structural pre-check that catches obvious issues
- A well-defined interface for the actual tool
- Environment variable `FRET_REALIZABILITY_CMD` to configure the tool

When the tool becomes available:
1. Set `FRET_REALIZABILITY_CMD` to the correct command
2. The runner will invoke it with `--spec roverSpec_full.fcs --signals signals_full.yaml`
3. Parse exit code (0 = realizable) and stdout for diagnostics

## Design decisions

- **YAML for requirements**: chosen over JSON for readability and comments
- **FCS for export**: the `.fcs` format is text-based for easy diffing
- **Signal inventory as YAML**: single source of truth, easy to extend
- **Separation of concerns**: each module does one thing, composable via CLI
- **Verified vs unverified signals**: explicitly tracked to prevent
  requirements from depending on non-existent infrastructure
