# spacetry_monitors — FRETish Writing and Verification Agent

A toolchain for transforming natural-language mission descriptions into
minimal, verifiable FRETish safety requirements for RiTMOS monitor synthesis
in the SpaceTry testbed.

## Quick start

```bash
# From the spacetry_monitors directory:
cd src/spacetry_monitors

# View the signal inventory
python -m fretish_agent.cli inventory

# Run full analysis on an example scenario
python -m fretish_agent.cli advise examples/scenario_rock_survey/requirements.yaml

# Export RiTMOS artifacts
python -m fretish_agent.cli export examples/scenario_rock_survey/requirements.yaml -o /tmp/output

# Lint requirements
python -m fretish_agent.cli lint examples/scenario_rock_survey/requirements.yaml

# Check signal availability
python -m fretish_agent.cli check-signals examples/scenario_safe_return/requirements.yaml
```

## Requirements

- Python 3.10+
- PyYAML (`pip install pyyaml`)

## Commands

| Command          | Description                                          |
|------------------|------------------------------------------------------|
| `inventory`      | Print all known SpaceTry signals                     |
| `lint`           | Validate requirement syntax and semantics            |
| `check-signals`  | Verify all referenced signals exist in the system    |
| `realize`        | Run realizability checking (placeholder if no tool)  |
| `export`         | Export `signals_full.yaml` + `roverSpec_full.fcs`    |
| `advise`         | Full pipeline: lint → check → realize → advise       |

## Workflow

1. Describe your mission scenario in natural language.
2. Using `config/agent_prompt.md` and `config/fretish_templates.yaml` as
   references, write a `requirements.yaml` with the minimal safety rules.
3. Run `fretish-agent advise requirements.yaml` to validate.
4. Iterate on the requirements until lint/signals/realizability pass.
5. Run `fretish-agent export requirements.yaml -o output/` to generate
   the RiTMOS artifacts.

## Example scenarios

- **Rock Survey at Ridge** (`examples/scenario_rock_survey/`):
  Fully verified scenario using only implemented signals.
- **Safe Return Under Low Battery** (`examples/scenario_safe_return/`):
  Demonstrates how the agent flags unverified signals (battery).

## FRET realizability

The realizability checker is behind a placeholder. To enable:

```bash
export FRET_REALIZABILITY_CMD="fret-realizability --check"
```

Without this, the agent runs a structural pre-check that catches
obvious issues (missing outputs, conflicting requirements).

## Running tests

```bash
cd src/spacetry_monitors
python -m pytest tests/ -v
```

## Architecture

See `docs/architecture.md` for the full module structure and data flow.

## Open questions

1. **FRET realizability command**: What is the exact command to invoke
   FRET realizability checking in this environment?
2. **RiTMOS artifact schemas**: Are there canonical schema files or
   examples for `signals_full.yaml` and `roverSpec_full.fcs`?
3. **Battery signals**: When will `spacetry_battery` be implemented?
   Requirements referencing battery signals are currently flagged.
4. **Perception pipeline**: `sample_detected` and `hazard_nearby` are
   stubs — when will a real perception pipeline be available?
