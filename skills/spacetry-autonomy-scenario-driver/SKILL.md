---
name: spacetry-autonomy-scenario-driver
description: Create or update SpaceTry autonomy test scenarios, scenario-generation prompts, and scenario driver implementations that inject uncertainty and evaluate the rover's autonomy in simulation. Use when Codex needs to turn SpaceTry behavior trees, missions, monitors, world files, fault-model notes, or scenario prompt templates into a scenario plan, fault-to-object traceability, scenario-specific launch or config changes, execution guidance, or validation steps for autonomy evaluation in this repo.
---

# SpaceTry Scenario Driver

Use this skill to design or implement autonomy-evaluation scenarios for the SpaceTry rover.

## Workflow

1. Read `AGENTS.md` first.
2. Read package-specific `AGENTS.md` or `.instructions.md` files for every package you may modify.
3. Load `skills/spacetry-autonomy-scenario-driver/references/repo-map.md` for the usual files and decision points.
4. Inspect the source files and dependencies needed for the scenario:
   - behavior tree
   - mission config or mission docs
   - monitor definitions
   - world/model files
   - scenario docs in `scenarios/`
   - project documentation at `docs/`
5. Define the scenario contract before editing code:
   - autonomy aspect under test
   - uncertainty source and taxonomy
   - fault subject, attribute, manifestation, and domain
   - trigger conditions
   - measurements, logging signals, and success criteria
   - scenario configurable parameters
   - scenario execution
   - scenario execution report structure, expected contents, and store location at the host machine
6. Prefer adding scenario-specific artifacts over modifying baseline autonomy logic.
7. Validate with Docker-only ROS2/build/test commands.

## Non-Negotiables

- Treat the behavior tree and monitors as the baseline under evaluation. Do not change them unless the user explicitly approves a bug fix.
- During scenario-driver generation, avoid modifying rover ROS 2 packages unless a genuine implementation bug is found and the user approves the change.
- Run every ROS2, Gazebo, build, or simulation command in Docker.
- When a scenario touches `src/spacetry_world`, read that package's instructions and the models package's `src/spacetry_models` README.md before editing and re-run world validation if the world changes.
- During the scenario driver generation and testing, none of the existing markdown files should be modified. If you need to add information, documentation, or running instructions about the generated scenario, do it inside the driver implementation source folder or package.

## Scenario Contract

Write the scenario in terms of these fields:

- `autonomy_aspect`: perception, behavior flexibility, resource awareness, mission resilience, obstacle intelligence, or safety
- `uncertainty_location`: environment, model, adaptation functions, goals, managed system, or resources
- `uncertainty_nature`: epistemic or variability
- `fault_subject`: environment object or robot subsystem
- `fault_attribute`: the targetable property that changes
- `manifestation`: stuck, noisy, or degrading
- `space_domain`: isolated, contiguous, or scattered when the fault is spatial
- `time_domain`: transient, permanent, or intermittent when the fault is temporal
- `trigger`: simulation time or robot-state predicate
- `measurements`: adaptation speed, recovery rate, safety preservation, goal viability
- `success_outcome`: achieved, degraded, or failed
- `report`: the file with results of running the scenario, including metric values and logged signals

If there is not enough information to infer any of these fields from the mission description, BT, monitors, battery, perception and world packages, ask the user for clarification instead of making assumptions.

## Code Style and Guidelines

- Follow instructions provided by the ROS2 community, available in: https://docs.ros.org/en/rolling/The-ROS2-Project/Contributing/Code-Style-Language-Versions.html
- Follow additional instructions in `AGENTS.md` files from project-wide and package-specific (in the `src/` sub-folders).
- In case of doubt, conflicting or missing information, ask clarification from the user.

## Fault Mapping

Use `scenarios/space-fault-model.md` to map the fault model to implementation details for the scenario driver.

For each fault, the scenario driver maintains traceability:

1. Between fault subjects/attributes and their corresponding ROS 2 nodes and Gazebo objects
2. ROS 2 topic, parameter, service, node, launch arg, or config file that exposes it
3. Gazebo model, world element, or plugin involved
4. Trigger condition
5. Logging signal that proves the injection happened
6. Monitor or metric that detects the rover response

If traceability is incomplete, call that out explicitly instead of inventing nonexistent ROS/Gazebo hooks.

## Prompt Generation

When the user wants only a prompt or scenario description, structure it around:

- BT path
- implementation packages
- mission file
- monitors and safety constraints
- uncertainty injection design
- metric definitions
- expected outputs or reports

Use `scenarios/SCENARIO_PROMPT_TEMPLATE.md` for the full shape and `scenarios/SCENARIO_PROMPT_QUICK_REF.md` for a concrete example and naming conventions.

## Implementation Guidelines

Prefer the least invasive path:

1. The scenario driver should not make changes on the autonomy behavior being evaluated. New packages needed for implementing the scenario driver can be added inside `src/` and should be named `spacetry_scenario_<scenario_name>`. 
2. Only changes on the mission and world packages are allowed within the remaining `src/` sub-folders.
3. The scenario driver ROS 2 nodes should be launched from a new launch file inside its package sub-folder named `launch`, and the launch file should be named `scenario_<scenario_name>.launch.py`.  The launch file can also include the baseline mission launch file as a component. If the scenario requires changes to the mission structure, add a new mission config file and use it in the scenario launch file instead of the baseline one.

Keep scenario logic observable:

- log injection timestamps
- log the exact fault state applied
- log the rover response signal
- write metrics that match the scenario contract

## Execution Guidance

Use Docker for all execution. Use the commands below to build, run, and validate the scenario in a containerized ROS 2 environment. 
This ensures consistency and reproducibility across different host machines. 

### Verify Running and Built Container Images

If you need to stop the container, run the command:

```bash
docker compose -f docker/docker-compose.yaml down
```
 
Before running any command, verify if the docker container is built with:

```bash
docker compose -f docker/docker-compose.yaml images
```

If the docker container is not built, build it with:

```bash
bash scripts/build.sh
```

Then, verify if the docker container is running with:

```bash
docker compose -f docker/docker-compose.yaml ps
```

### Running the container image and building the project

If the docker container is not running, start it with:

```bash
bash scripts/run.sh
```

Make sure the project is built:

```bash
docker compose -f docker/docker-compose.yaml exec spacetry colcon build --event-handlers console_direct+
```

## Validation

Rebuild packages if scenario components have changed or new ones were added:

```bash
docker compose -f docker/docker-compose.yaml exec spacetry colcon build --merge-install --event-handlers console_direct+
```

If you changed `src/spacetry_world`, also run:

```bash
docker compose -f docker/docker-compose.yaml exec spacetry /ws/scripts/verify_world.sh
```

### Execute the scenario driver to generate the report
When the scenario driver implementation is complete, run the associated launch file created to execute the scenario and generate the report with the defined metrics and logged signals.

Use the project-wide running instructions for executing a launch file in Docker, and specify the scenario driver launch file path.

In the final answer, report:

- what scenario was created or updated
- what assumptions were made
- what was the baseline autonomy logic, intentionally left untouched, that was being evaluated
- which Docker validations ran
- any remaining traceability or observability gaps
