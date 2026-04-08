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
   - mission config or mission docs
   - monitor definitions
   - world/model files
   - project documentation at `docs/`
5. Define the scenario contract before editing code (see **[Scenario Contract](#scenario-contract)** section for all required fields).

### 2. Scenario Driver Parametrization (Implementation)

Follow the **[Implementation Guidelines](#implementation-guidelines)** and **[Code Style and Guidelines](#code-style-and-guidelines)** sections to implement the scenario driver.

1. Add scenario-specific artifacts instead of modifying baseline autonomy logic.
2. Create scenario package, launch file, and configuration as described in **[Implementation Guidelines](#implementation-guidelines)**.
3. Implement logging and observability per **[Logging and Observability](#logging-and-observability)**.

### 3. Scenario Driver Validation

Validate the implementation before execution using the **[Validation](#validation)** section.

1. Rebuild scenario packages with Docker (see **[Validation](#validation)**).
2. If changes were made to `src/spacetry_world`, run world verification (see **[Validation](#validation)**).

### 4. Scenario Driver Execution (Execution & Reporting)

Execute the scenario driver to generate the report with metrics and logging signals using the **[Execution Guidelines](#execution-guidelines)** and **[Logging and Observability](#logging-and-observability)** sections.

1. Follow **[Execution Guidelines](#execution-guidelines)** to verify Docker setup and build the project.
2. Launch the scenario driver and ensure logging signals are captured per **[Logging and Observability](#logging-and-observability)**.
3. Generate and write the final report with metrics, logged signals, and outcomes per **[Execution Guidelines](#execution-guidelines)** final answer guidelines.

## Non-Negotiables

Policies that must be followed during scenario driver generation, implementation, validation, and execution:

- Do not consider anything under the host `log/` folder as scenario input. This bind-mounted folder is reserved for scenario execution outputs such as reports, metrics artifacts, rosbags, and runtime logs.
- Treat the behavior tree and monitors as the baseline under evaluation. Do not change them unless the user explicitly approves a bug fix.
- During scenario-driver generation, avoid modifying rover ROS 2 packages unless a genuine implementation bug is found and the user approves the change.
- Run every ROS2, Gazebo, build, or simulation command in Docker.
- When a scenario touches `src/spacetry_world`, follow `src/spacetry_world/AGENTS.md` for world-specific constraints and validation.
- Do not modify existing Markdown files in the repository during scenario-driver generation, implementation, testing, or reporting. This includes templates, quick references, skill files, AGENTS files, guides, and READMEs.
- If the scenario requires new documentation, instructions, prompt text, or reports, keep implementation-facing Markdown in the new scenario package under `src/spacetry_scenario_<scenario_name>/`, but write execution outputs such as generated reports to the bind-mounted host `log/` folder.
- Treat repository Markdown files outside the new scenario package as read-only unless the user explicitly asks to edit them.

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

### Core Metrics

Define measurable metrics for autonomy evaluation:

- **Adaptation speed**: Time in milliseconds between uncertainty injection and rover reaction
- **Safety preservation**: Key-value pairs with safety constraints (from monitors) and boolean preservation state
- **Goal viability**: Key-value pairs with mission goal and boolean indicating goal viability
- **Recovery rate**: Time in milliseconds between rover reaction to triggered uncertainty and reaction outcome
- **Additional mission-specific metrics**: Any extra metrics requested by the user or needed for the scenario, each with an explicit unit or boolean status and a short description

If there is not enough information to infer any of these fields from the mission description, BT, monitors, battery, perception and world packages, ask the user for clarification instead of making assumptions.

## Code Style and Guidelines

- Follow instructions provided by the ROS2 community, available in: https://docs.ros.org/en/rolling/The-ROS2-Project/Contributing/Code-Style-Language-Versions.html
- Follow additional instructions in `AGENTS.md` files from project-wide and package-specific (in the `src/` sub-folders).
- In case of doubt, conflicting or missing information, ask clarification from the user.

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

## Implementation Guidelines

Prefer the least invasive path:

1. The scenario driver should not make changes on the autonomy behavior being evaluated. New packages needed for implementing the scenario driver can be added inside `src/` and should be named `spacetry_scenario_<scenario_name>`. 
2. Only changes on the mission and world packages are allowed within the remaining `src/` sub-folders.
3. The scenario driver ROS 2 nodes should be launched from a new launch file inside its package sub-folder named `launch`, and the launch file should be named `scenario_<scenario_name>.launch.py`.  The launch file can also include the baseline mission launch file as a component. If the scenario requires changes to the mission structure, add a new mission config file and use it in the scenario launch file instead of the baseline one.

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

Write scenario outputs to the bind-mounted host `log/` folder using a per-scenario subdirectory, for example:

- `log/scenario_<scenario_name>/scenario_<scenario_name>_report.md`
- `log/scenario_<scenario_name>/metrics/`
- `log/scenario_<scenario_name>/rosbags/`
- `log/scenario_<scenario_name>/runtime/`

Keep scenario logic observable:

- log injection timestamps
- log the exact fault state applied
- log the rover response signal
- write metrics that match the scenario contract

## Validation

Rebuild packages if scenario components have changed or new ones were added:

```bash
docker compose -f docker/docker-compose.yaml exec spacetry colcon build --merge-install --event-handlers console_direct+
```

If you changed `src/spacetry_world`, also run:

```bash
docker compose -f docker/docker-compose.yaml exec spacetry /ws/scripts/verify_world.sh
```

## Execution Guidelines

- Use Docker for all execution. Use the commands below to build, run, and validate the scenario in a containerized ROS 2 environment. This ensures consistency and reproducibility across different host machines. 

- Store the generated report in the bind-mounted host `log/` folder with the name `spacetry_scenario_{scenario_name}_report.md`, ideally inside `log/spacetry_scenario_{scenario_name}/`, so it is accessible from the host machine after running the scenario.

- If necessary, mount the `log/` folder as a volume in Docker (e.g., `-v $(pwd)/log:/ws/log` - from the repository root) to ensure that all outputs from the scenario execution, including rosbags, metrics files, runtime logs, and the final report, are saved to the host machine for analysis and record-keeping.

- Make sure all the folders and files needed for the report are written under bind-mounted Docker volumes so they are accessible from the host machine after running the scenario. This includes rosbags, derived metrics files, and runtime logs, all of which should be placed under the host `log/` folder.

- The created scenario driver package should be copied into the docker container image after it is built. Use: `docker cp $(pwd)/src/spacetry_spacetry_scenario_{scenario_name} docker-spacetry-1:/ws/src/`




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
