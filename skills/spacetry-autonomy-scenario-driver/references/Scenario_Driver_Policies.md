# Scenario Driver Policies

These policies must be followed during scenario driver generation, implementation, validation, and execution.

- Treat both `log/` and `logs/` as output-only directories.
- Before reusing any existing artifact, confirm all of the following:
  - the artifact path is under `src/` or another canonical source directory listed in the skill workflow
  - the artifact path is not under `log/` or `logs/`
  - the artifact is a maintained source artifact, not a generated execution output
  - if any of the above is unclear, do not reuse it
- Never read scenario code, launch files, configs, reports, generated packages, or package templates from `log/` or `logs/` in order to design, scaffold, restore, or implement a scenario driver.
- Files under `log/` or `logs/` may be inspected only to report execution results from the current run, not to recover prior implementations or bootstrap new ones.
- If a similarly named scenario exists only under `log/` or `logs/`, ignore it as implementation input and recreate the scenario from canonical repository sources unless the user explicitly asks to restore or migrate it.
- Before reusing any existing scenario artifact as a starting point, verify that it lives under `src/`, `docs/`, `skills/spacetry-autonomy-scenario-driver/assets/`, or `skills/spacetry-autonomy-scenario-driver/references/`. If it does not, do not reuse it.
- Treat the behavior tree and monitors as the baseline under evaluation. Do not change them unless the user explicitly approves a bug fix.
- During scenario-driver generation, avoid modifying rover ROS 2 packages unless a genuine implementation bug is found and the user approves the change.
- Run every ROS2, Gazebo, build, or simulation command in Docker.
- Because the repository source tree is not bind-mounted into `/ws/src`, copy any new or modified scenario package into the running container before building or launching it.
- When a scenario touches `src/spacetry_world`, follow `src/spacetry_world/AGENTS.md` for world-specific constraints and validation.
- Do not assume the world is empty. Before adding obstacles, hazards, or alternate goals, inspect the active world SDF and mission waypoint files to account for mission-relevant objects already present in the baseline map.
- Do not set deadlines, route expectations, or injection locations without checking that they are realistic for the baseline start pose, existing target placement, and existing obstacle field.
- Do not modify existing Markdown files in the repository during scenario-driver generation, implementation, testing, or reporting. This includes templates, quick references, skill files, AGENTS files, guides, and READMEs.
- If the scenario requires new documentation, instructions, prompt text, or reports, keep implementation-facing Markdown in the new scenario package under `src/spacetry_scenario_<scenario_name>/`, but write execution outputs such as generated reports to the bind-mounted host `log/` folder.
- Treat repository Markdown files outside the new scenario package as read-only unless the user explicitly asks to edit them.
- Scenario drivers must generate a final report not only on nominal completion but also on interrupted execution.
- If a scenario run is stopped by `SIGINT`, `SIGTERM`, launch shutdown, timeout, or user interruption, the driver must still write:
  - the scenario report Markdown file
  - the metrics JSON file
  - the runtime timeline or equivalent event log
- Interrupted-run reports must explicitly mark the termination reason as `interrupted`, `signal`, `timeout`, or equivalent.
- A scenario evaluation is not considered valid unless an interrupted run is tested and confirmed to produce the required report artifacts under the bind-mounted host `log/` folder.
