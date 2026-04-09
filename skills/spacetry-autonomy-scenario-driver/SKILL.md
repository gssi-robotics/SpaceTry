---
name: spacetry-autonomy-scenario-driver
description: Create or update SpaceTry autonomy test scenarios and report results for the execution of scenario driver implementations that inject uncertainty and evaluate the rover's autonomy in simulation. Use when Codex needs to turn SpaceTry behavior trees, missions, monitors, world files, fault-model notes, or scenario prompt templates into a scenario plan, fault-to-object traceability, scenario-specific launch or config changes, execution guidance, or validation steps for autonomy evaluation in this repo.
---

# SpaceTry Scenario Driver

Use this skill to design, implement, or execute autonomy-evaluation scenarios for the SpaceTry rover.
After the implementation, validate and execute the scenario driver in a Dockerized environment, and generate a report with the defined metrics and logged signals.

## Workflow

### 1. Scenario Driver Generation (Planning & Preparation)

Define the scenario before implementing it. Follow the **[Non-Negotiables](#non-negotiables)** section and the **[Scenario Contract](#scenario-contract)** section for detailed field definitions.

1. Read `AGENTS.md` first and review **[Non-Negotiables](#non-negotiables)**.
2. Follow the rule hierarchy from project and package-specific `AGENTS.md` or `.instructions.md` files for every package you may modify.
3. Load `skills/spacetry-autonomy-scenario-driver/references/repo-map.md` for the usual files and decision points.
4. Inspect the source files and dependencies needed for the scenario:
   - behavior tree
   - battery resources and perception sensors
   - bringup launch files and ROS 2 package-specific launch instructions when the scenario will include baseline bringup
   - mission config or mission docs
   - monitor definitions
   - world SDF files and related models files
   - project documentation at `docs/`
5. Build a baseline world-and-mission map before designing uncertainty injection:
   - compare the prompt or reference scenario goals against the existing mission waypoints and the active world SDF
   - identify which goals, landmarks, obstacles, and hazards already exist in the baseline world
   - estimate whether the launch point, route length, and mission deadline are mutually realistic in the existing map
   - if the prompt goal already matches an existing waypoint or world entity, treat that baseline target as the goal under evaluation instead of inventing a new one
6. Verify artifact provenance per the **[Non-Negotiables](#non-negotiables)** before reusing any existing scenario artifact.
7. Define the scenario contract before editing code (see **[Scenario Contract](#scenario-contract)** section for all required fields).

### 2. Scenario Driver Parametrization (Implementation)

Follow the **[Implementation Guidelines](#implementation-guidelines)** and **[Code Style and Guidelines](#code-style-and-guidelines)** sections to implement the scenario driver.

1. Add scenario-specific artifacts instead of modifying baseline autonomy logic.
2. Create scenario package, launch file, and configuration as described in **[Implementation Guidelines](#implementation-guidelines)**.
3. Implement logging and observability per **[Logging and Observability](#logging-and-observability)**.

### 3. Scenario Driver Validation

Validate the implementation before execution using the **[Validation](#validation)** section.

1. Sync the scenario package from the host repository into `/ws/src` inside the running container before rebuilding.
2. Rebuild scenario packages with Docker (see **[Validation](#validation)**).
3. If changes were made to `src/spacetry_world`, run world verification (see **[Validation](#validation)**).

### 4. Scenario Driver Execution (Execution & Reporting)

Execute the scenario driver to generate the report with metrics and logging signals using the **[Execution Guidelines](#execution-guidelines)** and **[Logging and Observability](#logging-and-observability)** sections.

1. Follow **[Execution Guidelines](#execution-guidelines)** to verify Docker setup and build the project.
2. Launch the scenario driver and ensure logging signals are captured per **[Logging and Observability](#logging-and-observability)**.
3. Generate and write the final report with metrics, logged signals, and outcomes per **[Execution Guidelines](#execution-guidelines)** final answer guidelines.

## Non-Negotiables

Policies that must be followed during scenario driver generation, implementation, validation, and execution:

- Treat both `log/` and `logs/` as output-only directories.
- Before reusing any existing artifact, confirm all of the following:
   - the artifact path is under `src/` or another canonical source directory listed above
   - the artifact path is not under `log/` or `logs/`
   - the artifact is a maintained source artifact, not a generated execution output
   - if any of the above is unclear, do not reuse it.
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


## Scenario Contract

Write the scenario in terms of these fields:

- `autonomy_aspect`: perception, behavior flexibility, resource awareness, mission resilience, obstacle intelligence, or safety
- `uncertainty_location`: environment, model, adaptation functions, goals, managed system, or resources
- `fault_subject`: environment object or robot subsystem
- `fault_attribute`: the targetable property that changes
- `manifestation`: stuck, noisy, or degrading
- `space_domain`: isolated, contiguous, or scattered when the fault is spatial
- `time_domain`: transient, permanent, or intermittent when the fault is temporal
- `trigger`: simulation time or robot-state predicate
- `core_metrics`: Details below under **Core Metrics**
- `additional_metrics`: mission-specific metrics requested by the user or needed for the scenario
- `outcome_assessment`: PASS, DEGRADED, or FAIL
- `report`: the generated Markdown file under the bind-mounted host `log/` folder with the results of running the scenario, including metric values and logged signals
- `baseline_map_assessment`: a short statement of which baseline goals and obstacles already exist in the active world and how they affect route feasibility, deadline realism, and uncertainty placement
- `fault_attribution_rule`: explicit rule for deciding whether a detection or reaction is attributable to the injected uncertainty rather than baseline hazards, nominal behavior, or unrelated monitor violations
- `control_rationale_rule`: explicit rule for how the scenario records and classifies control-command changes such as `goal_alignment`, `obstacle_avoidance`, `replan_execution`, `monitor_enforcement`, or `unknown`

### Core Metrics

Define measurable metrics for autonomy evaluation:

- **Adaptation speed**: Time in milliseconds between uncertainty injection and the first attributable rover reaction
- **Reaction attribution status**: Boolean indicating whether the credited reaction can be distinguished from nominal behavior, baseline hazards, or unrelated monitor-triggered behavior
- **Control rationale at reaction**: The logged rationale attached to the credited control response, such as `goal_alignment`, `obstacle_avoidance`, `replan_execution`, `monitor_enforcement`, or `unknown`
- **Safety preservation**: Key-value pairs with safety constraints (from monitors) and boolean preservation state
- **Goal viability**: Key-value pairs with mission goal and boolean indicating goal viability
- **Recovery rate**: Time in milliseconds between rover reaction to triggered uncertainty and reaction outcome
- **Detection attribution status**: Boolean indicating whether the credited detection can be distinguished from baseline hazards, unrelated obstacle signals, or other confounding runtime events
- **Minimum fault distance at detection**: Rover-to-injected-fault distance in meters when detection is credited
- **Baseline-confound status**: Boolean or short string indicating whether a baseline obstacle, monitor violation, or unrelated runtime condition could explain the credited detection or reaction
- **Additional mission-specific metrics**: Any extra metrics requested by the user or needed for the scenario, each with an explicit unit or boolean status and a short description

Fault or reaction detections must satisfy scenario-specific attribution checks such as expected sensing range, relative geometry, and consistency with the injected fault subject.

Motion reactions must not be credited from `cmd_vel` deviation alone. A credited rover reaction must be supported by a logged control rationale and by scenario state that distinguishes injected-fault response from nominal goal alignment, baseline hazards, monitor enforcement, or unrelated runtime behavior.

If attribution is ambiguous, report the relevant detection or reaction metric as `AMBIGUOUS` or `NOT ATTRIBUTABLE` instead of presenting a numeric value as though it were confidently caused by the injected uncertainty.

If there is not enough information to infer any of these fields from the mission description, BT, monitors, battery, perception and world packages, ask the user for clarification instead of making assumptions.

## Code Style and Guidelines

- Follow instructions provided by the ROS2 community, available in: https://docs.ros.org/en/rolling/The-ROS2-Project/Contributing/Code-Style-Language-Versions.html
- Follow additional instructions in `AGENTS.md` files from project-wide and package-specific (in the `src/` sub-folders).
- In case of doubt, conflicting or missing information, ask clarification from the user.

### Scenario Naming Convention

For consistency, name scenarios:
```
spacetry_scenario_{autonomy_aspect}_{uncertainty_type}_{intensity}
```

**Examples:**
- `spacetry_scenario_perception_lidar_degradation_gradual`
- `spacetry_scenario_navigation_dynamic_obstacles_dense`
- `spacetry_scenario_mission_power_constraints_critical`
- `spacetry_scenario_safety_cascading_failures_multiple`

## Uncertainty and Fault Mapping

Use `skills/spacetry-autonomy-scenario-driver/references/Uncertainty_Taxonomy.md` to map the uncertainty and associated fault to implementation details for the scenario driver.

For each fault, the scenario driver maintains traceability:

1. Between fault subjects/attributes and their corresponding ROS 2 nodes and Gazebo objects
2. ROS 2 topic, parameter, service, node, launch arg, or config file that exposes it
3. Gazebo model, world element, or plugin involved
4. Trigger condition and timing for injection
5. Logging signal that proves the injection happened
6. Monitor or metric that detects the rover response

If traceability is incomplete, call that out explicitly instead of inventing nonexistent ROS/Gazebo hooks.

Do not claim that the injected uncertainty was detected or that the rover reacted to it unless the report includes explicit evidence that separates the injected fault from baseline hazards, pre-existing obstacles, or unrelated monitor-triggered behavior.

If the first significant post-injection control change is labeled `goal_alignment`, `unknown`, or another non-attributable rationale, do not credit it as adaptation to the injected fault.

## Implementation Guidelines

Prefer the least invasive path:

1. The scenario driver should not make changes on the autonomy behavior being evaluated. New packages needed for implementing the scenario driver can be added inside `src/` and should be named `spacetry_scenario_<scenario_name>`. 
2. Only changes on the mission and world packages are allowed within the remaining `src/` sub-folders.
3. The scenario driver ROS 2 nodes should be launched from a new launch file inside its package sub-folder named `launch`, and the launch file should be named `scenario_<scenario_name>.launch.py`.  The launch file can also include the baseline bringup unchanged launch file as a component. If the scenario requires changes to the mission structure, add a new mission config file and use it in the scenario launch file instead of the baseline one.
4. When including baseline bringup or depending on its launch arguments, follow `src/spacetry_bringup/AGENTS.md` for launch-integration, `use_sim_time`, BT runner, parameter-wiring, and topic QoS compatibility rules.
5. Before any Dockerized build or launch of the scenario package, copy it into the running container:

```bash
docker cp $(pwd)/src/spacetry_scenario_{scenario_name} docker-spacetry-1:/ws/src/
```

Before choosing a trigger, injected obstacle pose, timeout, or derived goal logic:
- Inspect the active world SDF and mission waypoint files together.
- Reuse prompt-specified goals when they already correspond to baseline world entities or mission waypoints.
- Avoid injecting a new obstacle that duplicates or trivially overlaps an existing hazard unless the scenario explicitly intends to intensify that known obstacle field.
- Place runtime uncertainty relative to the nominal route that actually exists in the baseline map, not an assumed straight-line route through empty terrain.
- Scale mission deadlines and success windows to the baseline route length from the launch point to the evaluated goal.


### Component Specification

Specify what the driver component should perform:

- **Monitor** the rover's execution state (position, battery, sensor status)
- **Trigger** uncertainty events at decision-critical moments (e.g., when rover commits to action)
- **Measure** adaptation success (goal completion, safety violations, contingency activations)
- **Adapt injection intensity** based on observed autonomy performance (gradually increase challenge)
- **Log behavior** for post-execution analysis (decision timestamps, fallback activations, constraint violations)

### Behavior Configuration

For each autonomy and safety requirement being evaluated, specify injection strategy:

**1. Injection Timing** — When uncertainty is triggered relative to rover behavior:
- At decision point of the behavior tree
- Mid-action (during active execution)
- During contingency (fallback paths)

**2. Intensity Increase** — How uncertainty level evolves during execution:
- Gradual (progressive increase over time)
- Sudden (immediate step change)
- Cascading (multiple overlapping uncertainties)

### Logging and Observability

- The logging signals and events related to the scenario should be stored in a rosbag under the bind-mounted host `log/` folder and included in the final report with the metric values and scenario outcomes.
- Use ROS 2 logging and rosbag existing solutions when possible, and avoid adding new logging topics or custom solutions that are not already part of the rover's ROS 2 packages.
- If there are any gaps in observability, call them out explicitly instead of inventing nonexistent ROS/Gazebo hooks. 
- Before depending on an existing SpaceTry topic for scenario triggers, derived mission state, or report metrics, confirm that the scenario node subscription QoS matches the publisher QoS used in the stack. In case of doubt, use an explicitly compatible QoS profile instead of the default subscription QoS. If there is a mismatch, treat that as an observability/integration issue to fix before evaluating the autonomy behavior.

Write scenario outputs to the bind-mounted host `log/` folder using the following per-scenario subdirectories:

- `log/scenario_<scenario_name>/scenario_<scenario_name>_report.md`
- `log/scenario_<scenario_name>/metrics/`
- `log/scenario_<scenario_name>/rosbags/`
- `log/scenario_<scenario_name>/runtime/`

Keep scenario logic observable:

- log injection timestamps
- log the exact fault state applied
- log the rover response signal
- write metrics that match the scenario contract
- log the injected fault pose, time, and unique identifier
- log rover pose at every credited detection and credited reaction event
- log rover-to-fault distance when detection or reaction is credited
- log which sensor, topic, or monitor signal produced each detection or reaction candidate
- log each significant `cmd_vel` change with its command values and an explicit control rationale such as `goal_alignment`, `obstacle_avoidance`, `replan_execution`, `monitor_enforcement`, or `unknown`
- log the scenario state used to classify each `cmd_vel` rationale, such as obstacle flags, monitor status, planner state, or BT state when available
- log whether each credited detection passed the scenario's `fault_attribution_rule`
- log whether each credited reaction passed the scenario's `fault_attribution_rule`
- log whether any baseline obstacle, monitor violation, or other confounding condition was active when a detection or reaction candidate was evaluated
- if a detection or reaction candidate is rejected, log the rejection reason

Prefer attribution-aware event names when scenario outputs include a runtime timeline or event log, for example:

- `fault_injected`
- `fault_detection_candidate`
- `fault_detection_attributed`
- `fault_detection_rejected`
- `reaction_candidate`
- `reaction_attributed`
- `reaction_rejected_due_to_baseline_hazard`
- `baseline_monitor_violation_active`
- `cmd_vel_changed`
- `control_rationale_classified`

## Validation

Rebuild the scenario driver related packages if scenario components have changed or new ones were added:

If the scenario related packages are new or has changed on the host, copy it into the running container first:

```bash
docker cp $(pwd)/src/spacetry_scenario_{scenario_name} docker-spacetry-1:/ws/src/
```

Use bootstrap build mode when `/ws/install/setup.bash` does not exist yet, such as the first build in a fresh container:

```bash
docker compose -f docker/docker-compose.yaml exec spacetry bash -lc "source /opt/ros/spaceros/setup.bash && source /etc/profile && colcon build --merge-install --event-handlers console_direct+"
```

Use overlay build mode after the workspace install tree already exists:

```bash
docker compose -f docker/docker-compose.yaml exec spacetry bash -lc "source /opt/ros/spaceros/setup.bash && source /etc/profile && source /ws/install/setup.bash && colcon build --merge-install --event-handlers console_direct+"
```

If you changed `src/spacetry_world`, also run:

```bash
docker compose -f docker/docker-compose.yaml exec spacetry bash -lc "source /opt/ros/spaceros/setup.bash && source /etc/profile && source /ws/install/setup.bash && /ws/scripts/verify_world.sh"
```

## Execution Guidelines

- Use Docker for all execution. Use the commands below to build, run, and validate the scenario in a containerized ROS 2 environment. This ensures consistency and reproducibility across different host machines. 
- Agents MUST rebuild the container image with `bash scripts/build.sh` before scenario execution unless the user explicitly says to reuse an existing image.
- Before executing a scenario launch, confirm the scenario package has been copied into `/ws/src` and that the workspace has already been built so `/ws/install/setup.bash` exists.

- Validate incrementally, starting with the launch and adjust the launch configuration and parameters if needed. Then move into the full scenario uninterruped execution. 

- Store the generated report in the bind-mounted host `log/` folder with the name `spacetry_scenario_{scenario_name}_report.md`, ideally inside `log/spacetry_scenario_{scenario_name}/`, so it is accessible from the host machine after running the scenario.

- If necessary, mount the `log/` folder as a volume in Docker (e.g., `-v $(pwd)/log:/ws/log` - from the repository root) to ensure that all outputs from the scenario execution, including rosbags, metrics files, runtime logs, and the final report, are saved to the host machine for analysis and record-keeping.

- Make sure all the folders and files needed for the report are written under bind-mounted Docker volumes so they are accessible from the host machine after running the scenario. This includes rosbags, derived metrics files, and runtime logs, all of which should be placed under the host `log/` folder.

- The created or updated scenario driver package should be copied into the running Docker container before it is built or executed. Use: `docker cp $(pwd)/src/spacetry_scenario_{scenario_name} docker-spacetry-1:/ws/src/`

### Execute the scenario driver to generate the report

When the scenario driver implementation is complete, run the associated launch file created to execute the scenario and generate the report with the defined metrics and logged signals.

Use the project-wide Docker and execution instructions from `AGENTS.md`, and specify the scenario driver launch file path.

In the final answer, report in a markdown format the following information:

- Which tasks were performed
- What scenario was created or updated
- What assumptions were made
- What was the baseline autonomy logic, intentionally left untouched, that was being evaluated
- Which Docker validations ran
- Any remaining traceability or observability gaps

The generated execution report should be placed under the bind-mounted host `log/` folder, preferably at `log/spacetry_scenario_{scenario_name}/spacetry_scenario_{scenario_name}_report.md`.
