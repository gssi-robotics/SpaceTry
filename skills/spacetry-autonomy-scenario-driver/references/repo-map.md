# SpaceTry Autonomy Scenario Driver Repo Map

Load this file when you need a quick map of where scenario-relevant information lives.

## Core inputs

- `AGENTS.md`
  Project-wide rules, Docker-only execution, autonomy-evaluation workflow.
- `skills/spacetry-autonomy-scenario-driver/assets/SCENARIO_PROMPT_TEMPLATE.md`
  Full scenario prompt structure and metric vocabulary.
- `skills/spacetry-autonomy-scenario-driver/references/SCENARIO_PROMPT_QUICK_REF.md`
  Short prompt patterns, naming convention, integration checklist.
- `skills/spacetry-autonomy-scenario-driver/references/space-fault-model.md`
  Fault subject, domain, trigger, manifestation, and traceability model.
- `skills/spacetry-autonomy-scenario-driver/references/Uncertainty_Taxonomy.md`
  Uncertainty dimensions and sources.

## Runtime and autonomy sources

- `src/spacetry_bt/trees/`
  Behavior trees and decision points for injection timing.
- `src/spacetry_mission/`
  Mission goals, configs, and expected waypoint or task structure.
- `src/spacetry_monitors/`
  Safety constraints and monitorable properties.
- `src/spacetry_perception/`
  Sensor-processing entry points and degradation targets.
- `src/spacetry_battery/`
  Resource-awareness hooks and low-power behavior signals.
- `src/spacetry_world/worlds/mars_outpost.sdf`
  Environment-level uncertainty insertion points.
- `src/spacetry_world/AGENTS.md`
  Extra rules for world edits and validation.

## Decision reminders

- Focus on scenario artifacts over edits to baseline autonomy logic.
- Treat BT and safety monitor as the thing being evaluated, not the artifacts to rewrite.
- If a scenario requires changing rover ROS2 packages during scenario generation, stop and ask for approval.
- Never run ROS2, Gazebo, or build commands on the host; use Docker.
- If you do not have enough information for a decision concerning the provided taxonomies, ask for input or clarification.
