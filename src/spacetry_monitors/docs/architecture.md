# FRETish Agent — Architecture

## Overview

The FRETish agent transforms natural-language mission/scenario descriptions
into minimal, verifiable safety requirements in FRETish form, suitable for
RiTMOS monitor synthesis. It integrates directly with NASA FRET for
realizability checking and with RiTMOS for Copilot monitor generation.

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
│   ├── realizability_runner.py # FRET CLI realizability integration
│   ├── revision_advisor.py    # Diagnose failures & suggest fixes
│   └── ritmos_exporter.py     # Export RiTMOS + FRET artifacts
├── config/
│   ├── signal_inventory.yaml  # Canonical signal ground truth
│   ├── fretish_templates.yaml # Common requirement patterns
│   └── agent_prompt.md        # System prompt for LLM-assisted mode
├── examples/
│   ├── scenario_rock_survey/  # Example 1: full end-to-end
│   └── scenario_safe_return/  # Example 2: with unverified signals
├── tests/                     # Test suite
├── docs/                      # This documentation
├── package.xml                # ROS 2 package manifest
├── CMakeLists.txt             # Build file
└── setup.py                   # Python package setup
```

## Integration with FRET and RiTMOS

### FRET (deps/fret)

NASA FRET provides:
- **FRETish formalization**: `fretcli formalize '<sentence>' -l pt` converts
  FRETish natural language to past-time LTL (ptLTL).
- **Realizability checking**: `fretcli realizability <project> <component>`
  checks whether an implementation can satisfy all requirements given any
  environment input.
- **Diagnosis**: `--diagnose` identifies minimal conflicting requirement sets.

FRET uses **Kind2** or **JKind** as the underlying model checker, with **Z3**
as the SMT solver.

### RiTMOS (deps/ritmos)

RiTMOS translates requirements into runtime monitors:
1. Reads a `.fcs` JSON file with ptLTL formulas
2. Reads a `signals.yaml` mapping signal names to ROS topics
3. Generates Copilot (Haskell) specs → compiles to C
4. Wraps in a ROS 2 node that subscribes to topics and runs `step()`

## Output artifact formats

### spec.fcs (RiTMOS input)

JSON format matching `deps/ritmos/inputs/sample.fcs`:

```json
{
  "robotSpec": {
    "Internal_variables": [],
    "Other_variables": [],
    "Functions": [],
    "Requirements": [
      {
        "name": "REQ_OBSTACLE_STOP",
        "fretish": "if obstacle_too_close the rover shall immediately satisfy cmd_move_stop",
        "ptLTL": "(H (obstacle_too_close -> cmd_move_stop))",
        "CoCoSpecCode": "",
        "PCTL": ""
      }
    ]
  }
}
```

FRET convention: requirement IDs containing "assumption" are treated as
environment assumptions during realizability checking.

### signals.yaml (RiTMOS input)

Flat format matching `deps/ritmos/inputs/signals.yaml`:

```yaml
signals:
  - {name: obstacle_min_range, type: float64, topic: /scan}
  - {name: cmd_move_stop, type: bool, topic: /move_stop}
numeric_bools:
  - {name: obstacle_too_close, expr: "obstacle_min_range < 1.0"}
```

### fretRequirementsVariables.json (FRET import/export)

Full FRET format matching `deps/ritmos/examples/robot_box/specs/fretRequirementsVariables.json`:

```json
{
  "requirements": [
    {
      "reqid": "REQ_OBSTACLE_STOP",
      "project": "SpaceTry",
      "fulltext": "if obstacle_too_close the rover shall immediately satisfy cmd_move_stop",
      "semantics": {
        "type": "nasa",
        "timing": "immediately",
        "response": "satisfaction",
        "variables": ["obstacle_too_close", "cmd_move_stop"],
        "component_name": "rover",
        "pt": "(H (obstacle_too_close -> cmd_move_stop))"
      }
    }
  ],
  "variables": [
    {
      "variable_name": "obstacle_too_close",
      "dataType": "boolean",
      "idType": "Internal",
      "component_name": "rover"
    }
  ]
}
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
┌──────────────────┐     ┌──────────────────┐
│  FRET CLI        │────►│  fretcli         │
│  formalize       │     │  formalize       │
│  FRETish → ptLTL │     │  -l pt           │
└────────┬─────────┘     └──────────────────┘
         │
         ▼
┌──────────────────┐     ┌──────────────────┐
│  realizability   │────►│  fretcli         │
│  _runner         │     │  realizability   │
│                  │     │  --diagnose      │
└────────┬─────────┘     └──────────────────┘
         │
         ▼
┌──────────────────┐
│ revision_advisor │  Suggests fixes if unrealizable
└────────┬─────────┘
         │
         ▼
┌──────────────────┐     ┌─────────────────────────────┐
│ ritmos_exporter  │────►│ spec.fcs (JSON)              │
│                  │     │ signals.yaml (RiTMOS)        │
│                  │     │ fretRequirementsVariables.json│
└──────────────────┘     └──────────┬──────────────────┘
                                    │
                                    ▼
                          ┌──────────────────┐
                          │  ritmos ros      │
                          │  package         │
                          │  --compile-      │
                          │  copilot         │
                          └──────────────────┘
```

## Setting up FRET realizability

1. Install FRET:
   ```bash
   cd deps/fret/fret-electron
   npm install
   npm run build-cli
   ```

2. Install solvers:
   - Z3 v4.14.1: https://github.com/Z3Prover/z3/releases
   - Kind2 v2.2.0 (NOT v2.3.0): https://github.com/kind2-mc/kind2
   - Or JKind: https://github.com/andrewkatis/jkind-1/releases

3. Add all binaries to PATH.

4. Import requirements into FRET:
   - Load `fretRequirementsVariables.json` via FRET GUI
   - Complete variable mapping for component "rover"

5. Run realizability:
   ```bash
   fretcli realizability SpaceTry rover --diagnose --json
   ```

## Setting up RiTMOS monitor generation

1. Build RiTMOS Docker image:
   ```bash
   cd deps/ritmos && docker compose build
   ```

2. Generate monitor package:
   ```bash
   docker compose run --rm ritmos \
     ritmos ros package inputs/spec.fcs \
       --pkg spacetry_monitor \
       --outdir ros_ws/src \
       --signals-map inputs/signals.yaml \
       --compile-copilot
   ```

3. Build and run:
   ```bash
   cd ros_ws && colcon build --packages-select spacetry_monitor
   source install/setup.bash
   ros2 run spacetry_monitor copilot_monitor_node
   ```

## CLI commands

| Command     | Description                                              |
|-------------|----------------------------------------------------------|
| `inventory` | Print all known SpaceTry signals                         |
| `lint`      | Validate requirement syntax and semantics                |
| `check-signals` | Verify all referenced signals exist in the system   |
| `realize`   | Run FRET realizability checking (or structural fallback) |
| `formalize` | Convert FRETish text to ptLTL via FRET CLI               |
| `export`    | Export spec.fcs + signals.yaml + fretRequirementsVariables.json |
| `ritmos`    | Export and show RiTMOS invocation commands                |
| `advise`    | Full pipeline: lint → check → realize → advise           |

## Design decisions

- **YAML for authoring** (requirements.yaml): readable, commentable
- **JSON for export** (.fcs, fretRequirementsVariables.json): matches FRET/RiTMOS
- **Signal inventory as YAML**: single source of truth, easy to extend
- **Assumption convention**: FRET treats IDs containing "assumption" as environment assumptions
- **Verified vs unverified signals**: explicitly tracked; requirements referencing unverified signals get warnings
- **Numeric bools in signals.yaml**: derived boolean predicates (like `obstacle_min_range < 1.0`) are exported as RiTMOS `numeric_bools` entries
