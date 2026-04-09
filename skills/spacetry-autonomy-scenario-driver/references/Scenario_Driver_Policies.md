# Scenario Driver Policies

These policies must be followed during scenario driver generation, implementation, validation, and execution.

## Mandatory Enforcement

These policies are mandatory, not advisory.

- Agents must treat policy compliance as a hard gate before any repository exploration, implementation, or validation work.
- If a planned read, search, or reuse step would touch `log/` or `logs/` for implementation input, the agent must not proceed with that step.
- The correct behavior is fail-closed:
  - stop using that path as input
  - discard it as implementation context
  - restart from canonical sources
- A policy violation must not be normalized after the fact as harmless context gathering. It is a defect in the scenario-generation workflow and should be explicitly acknowledged and corrected.

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
- Before using any repository-wide search command for planning or implementation, exclude `log/` and `logs/` from the search scope.
- Use search patterns that are safe by construction, for example:
  - `rg --glob '!log/**' --glob '!logs/**' <pattern>`
  - `rg --files src skills docs`
  - `rg --files -g 'AGENTS.md' -g '.instructions.md' src skills docs`
  - `find src skills docs -...`
- Do not use broad repository scans that include `log/` or `logs/` unless the task is specifically to inspect current-run outputs.
- If the user asks for implementation work and the only matching artifact is under `log/` or `logs/`, treat that as insufficient source provenance and recreate the implementation from canonical inputs.
- Treat the behavior tree and monitors as the baseline under evaluation. Do not change them unless the user explicitly approves a bug fix.
- Treat the BT runner configuration as part of the baseline autonomy under evaluation. Scenario launches must not override BT-runner-related launch arguments or parameters such as `tree_file`, BT params files, tick rate, or equivalent runner settings unless the user explicitly approves a baseline bug fix.
- During scenario-driver generation, avoid modifying rover ROS 2 packages unless a genuine implementation bug is found and the user approves the change.
- Run every ROS2, Gazebo, build, or simulation command in Docker.
- Because the repository source tree is not bind-mounted into `/ws/src`, copy any new or modified scenario package into the running container before building or launching it.
- When a scenario touches `src/spacetry_world`, follow `src/spacetry_world/AGENTS.md` for world-specific constraints and validation.
- Do not assume the world is empty. Before adding obstacles, hazards, or alternate goals, inspect the active world SDF and mission waypoint files to account for mission-relevant objects already present in the baseline map.
- Do not set deadlines, route expectations, or injection locations without checking that they are realistic for the baseline start pose, existing target placement, and existing obstacle field.
- Treat baseline uncertainty as first-class evaluation context, not merely confound noise. Scenario design must explicitly identify which uncertainties already come from SpaceTry source code, configuration, monitors, perception limits, and world hazards before adding new injected ones.
- If a baseline uncertainty is expected to exercise autonomy before the injected fault does, the scenario must record that separately instead of folding it into injected-fault evidence.
- Scenario reports must classify the observed maneuver even when it is caused only by baseline conditions. Do not collapse a classifiable baseline obstacle-avoidance or monitor-enforcement maneuver into `unknown` just because the injected uncertainty did not receive attribution credit.
- If the rover never reaches the route segment, timing condition, or system state needed for the injected uncertainty to matter, classify the injected-autonomy outcome as `UNTESTED` rather than `FAIL`.
- Do not declare injected-autonomy failure solely because the mission ended or was interrupted before the rover encountered the injected uncertainty.
- Treat `encountered` and `meaningfully evaluable` as separate gates. An encountered injected uncertainty does not automatically justify a `FAIL` classification if too little time remains afterward to observe detection, reaction, or recovery evidence.
- Scenario drivers must compute and report `evaluation_window_after_encounter_s`.
- Injected-autonomy `FAIL` is allowed only when the rover encountered the injected uncertainty and the run still provided enough post-encounter observation time to satisfy the contract's `meaningful_evaluation_rule`.
- If encounter happens too close to timeout or termination for fair evaluation, classify the injected outcome as `UNTESTED` or `INCONCLUSIVE`, not automatically `FAIL`.
- Scenario drivers must log trigger-decision diagnostics at injection time, including:
  - why injection was allowed
  - remaining mission time at injection
  - progress ratio at injection
  - whether baseline monitors were already active
- Scenario drivers must report all of the following reaction-side fields separately:
  - `observed_control_rationale`
  - `reaction_scope`
  - `reaction_attribution_status`
  - `active_context_at_reaction`
- The scenario runtime and timeout budget must be at least as long as the baseline BT evaluation horizon. A scenario must not shorten the run below the time the baseline BT and mission need to exercise the autonomy being evaluated.
- When a scenario depends on both baseline and injected uncertainties, the final report must distinguish:
  - what baseline uncertainty was exercised
  - what injected uncertainty was exercised
  - whether the injected uncertainty was actually encountered
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

## Recovery From Violation

If an agent accidentally reads from `log/` or `logs/` during scenario design or implementation:

- do not reuse any implementation detail learned only from those directories
- restart the affected reasoning step from canonical sources
- mention the violation and the corrective action in the final answer if it materially affected the work
- for any new repository-wide search after the violation, use an explicit exclusion command such as `rg --glob '!log/**' --glob '!logs/**' <pattern>` or restrict the search root to `src/`, `skills/`, and `docs/`
