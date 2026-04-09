---
name: spacetry-autonomy-scenario-driver
description: Create or update SpaceTry autonomy test scenarios and report results for the execution of scenario driver implementations that evaluate both baseline SpaceTry uncertainties and additional injected uncertainties in simulation. Use when the LLM agent needs to turn SpaceTry behavior trees, missions, monitors, or world files into a scenario plan, uncertainty traceability, scenario-specific launch or config changes, execution guidance, or validation steps for autonomy evaluation in this repo.
---

# SpaceTry Scenario Driver

Use this skill to design, implement, or execute autonomy-evaluation scenarios for the SpaceTry rover. The driver should challenge the rover's autonomy by injecting uncertainties at critical moments and measuring the rover's response in terms of safety, adaptation, and mission success.

After the implementation, validate and execute the scenario driver in a Dockerized environment, and generate a report with the defined metrics and logged signals.

Use `SKILL.md` as the primary workflow document.
Load the reference documents below when you reach the relevant step:

- `references/Scenario_Driver_Policies.md` for mandatory policies and artifact provenance rules
- `references/Scenario_Contract.md` for required contract fields, metrics, attribution rules, and ambiguity handling
- `references/Observability_and_Attribution.md` for logging requirements, event naming, `cmd_vel` rationale tracking, and fault attribution evidence
- `references/Validation_and_Execution.md` for Docker build, validation, execution, interrupted-run validation, and final reporting expectations
- `references/repo-map.md` for the usual repository files and scenario-driver decision points
- `references/Uncertainty_Taxonomy.md` for uncertainty-to-fault mapping
- `references/Gazebo.md` when the scenario depends on Gazebo object, world, or simulator behavior details
- `references/SCENARIO_PROMPT_QUICK_REF.md` when the user is providing or refining a scenario prompt

## Workflow

### 1. Scenario Planning and Preparation

Define the scenario before implementing it. Load `references/Scenario_Driver_Policies.md` and `references/Scenario_Contract.md` before making implementation decisions.

1. Read `AGENTS.md` first and then load `skills/spacetry-autonomy-scenario-driver/references/Scenario_Driver_Policies.md`.
2. Follow the rule hierarchy from project and package-specific `AGENTS.md` or `.instructions.md` files for every package you may modify.
3. Load `skills/spacetry-autonomy-scenario-driver/references/repo-map.md` for the usual files and decision points.
4. Apply a policy gate before any repository-wide search:
   - treat `log/` and `logs/` as forbidden implementation inputs
   - limit early searches to canonical roots such as `src/`, `skills/`, `docs/`, and package instruction files
   - use safe-by-construction commands such as `rg --files src skills docs` or `rg --glob '!log/**' --glob '!logs/**' <pattern>`
   - if a search accidentally touches `log/` or `logs/`, discard that result as implementation context and restart from canonical sources
5. Inspect the source files and dependencies needed for the scenario:
   - behavior tree
   - battery resources and perception sensors
   - bringup launch files and ROS 2 package-specific launch instructions when the scenario will include baseline bringup
   - mission config or mission docs
   - monitor definitions
   - world SDF files and related models files
   - project documentation at `docs/`
6. Build a baseline world-and-mission map before designing uncertainty injection:
   - compare the prompt or reference scenario goals against the existing mission waypoints and the active world SDF
   - identify which goals, landmarks, obstacles, and hazards already exist in the baseline world
   - identify baseline uncertainties already present in the source system, including BT obstacle-avoidance behavior, monitor-triggering battery or speed constraints, known hazards, perception limits, and launch-time defaults that can affect the mission before any new injection occurs
   - estimate whether the launch point, route length, and mission deadline are mutually realistic in the existing map
   - require a practical reachability assessment before finalizing the injection pose or trigger:
     - estimate whether the rover is likely to physically reach the intended injection zone under realistic baseline behavior, not only under ideal mission geometry
     - account for likely baseline hazard avoidance, monitor-driven slowdowns, launch-time settling, and other non-nominal effects that may prevent the injected uncertainty from ever being encountered
   - if the prompt goal already matches an existing waypoint or world entity, treat that baseline target as the goal under evaluation instead of inventing a new one
   - separate which uncertainties are already exercised by the baseline source code and environment from which uncertainties will be newly injected by the scenario
7. Verify artifact provenance per `references/Scenario_Driver_Policies.md` before reusing any existing scenario artifact.
8. Load `skills/spacetry-autonomy-scenario-driver/references/Scenario_Contract.md` and define the scenario contract before editing code.
9. Load `skills/spacetry-autonomy-scenario-driver/references/Uncertainty_Taxonomy.md` when mapping the fault subject, uncertainty location, manifestation, or trigger to implementation details.
10. Define at design time how each scenario artifact is consumed by ROS:
   - keep scenario contract and scenario config YAML as plain files
   - pass file paths and runtime toggles to the scenario node only through inline launch dict ROS parameters
11. When the prompt includes one or more injected uncertainties, identify:
   - the primary evaluation target
   - any secondary injected uncertainties
   - whether the scenario is single-uncertainty or multi-uncertainty
   - whether any stated interaction hypothesis is intentional or omitted

### 2. Scenario Driver Implementation

Follow the implementation, code-style, and observability guidance in this file and the linked references.

1. Add scenario-specific artifacts instead of modifying baseline autonomy logic.
2. Create the scenario package, launch file, and configuration following the implementation guidance in this file.
3. Load `skills/spacetry-autonomy-scenario-driver/references/Observability_and_Attribution.md` before implementing metrics, runtime timelines, rosbag capture, or report generation.
4. When the scenario depends on Gazebo entities, spawning, or world behavior, load `skills/spacetry-autonomy-scenario-driver/references/Gazebo.md`.
5. Keep the launch-to-node interface unambiguous:
   - use inline launch dict parameters for the scenario driver node
   - pass paths like `scenario_config_file` and `scenario_contract_file` as string parameters
   - treat scenario YAML files as driver-parsed artifacts, not ROS params files
6. Preserve the policy gate during implementation:
   - do not open, grep, or copy from `log/` or `logs/` to scaffold code
   - if you need examples, use only canonical artifacts under `src/`, `skills/`, or `docs/`

### 3. Scenario Driver Validation

Validate the implementation before execution by loading `skills/spacetry-autonomy-scenario-driver/references/Validation_and_Execution.md`.

1. Sync the scenario package from the host repository into `/ws/src` inside the running container before rebuilding.
2. Rebuild scenario packages with Docker using the validation reference.
3. If changes were made to `src/spacetry_world`, run world verification using the validation reference.
4. Run an intentionally interrupted validation run using an in-container PID-targeted `SIGINT` method, and confirm that the required report artifacts are written.
5. During iterative tuning, if only the scenario package changed, use the lighter scenario-package-only validation loop from `references/Validation_and_Execution.md` instead of repeating unrelated earlier validation steps.

### 4. Scenario Execution and Reporting

Execute the scenario driver to generate the report with metrics and logging signals using `references/Validation_and_Execution.md` and `references/Observability_and_Attribution.md`.

1. Follow `references/Validation_and_Execution.md` to verify Docker setup and build the project.
2. Launch the scenario driver and ensure logging signals are captured per `references/Observability_and_Attribution.md`.
3. Generate and write the final report with metrics, logged signals, and separate baseline-vs-injected outcomes per `references/Validation_and_Execution.md`.
4. Report important confounders explicitly when they affect interpretation, such as baseline hazard avoidance, monitor dominance, launch-time settling delays, or other baseline conditions that plausibly explain the observed behavior.
5. Treat `UNTESTED` as a valid early iteration outcome during scenario tuning when the rover never meaningfully encounters the injected uncertainty. This is part of refining scenario reachability and attribution, not automatically a scenario-driver defect.
6. Only after implementation is complete may `log/` or `logs/` be inspected, and then only for current-run result reporting.

## Execution Orchestration

When the validation or execution workflow needs reliable PID tracking, signal delivery, or log polling, temporary helper scripts or shell wrappers are allowed as execution-only tooling.

- Use them only to orchestrate launch, interruption, cleanup, or artifact verification.
- Do not treat them as scenario implementation artifacts.
- Keep them short-lived and avoid storing them as maintained repository source unless the user explicitly asks for that.

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

Treat uncertainty as two layers throughout planning and reporting:

1. **Baseline uncertainties** already present in SpaceTry source code, launch defaults, monitors, mission configuration, perception, and world hazards.
2. **Injected uncertainties** introduced by the scenario driver during execution.

Within injected uncertainties, distinguish:

1. the **primary evaluation target** that the scenario is mainly intended to assess
2. any **secondary injected uncertainties** that are also introduced in the same run

Secondary injected uncertainties may be related or unrelated to the primary evaluation target.

For each fault, the scenario driver maintains traceability:

1. Between fault subjects/attributes and their corresponding ROS 2 nodes and Gazebo objects
2. ROS 2 topic, parameter, service, node, launch arg, or config file that exposes it
3. Gazebo model, world element, or plugin involved
4. Trigger condition and timing for injection
5. Logging signal that proves the injection happened
6. Monitor or metric that detects the rover response

If traceability is incomplete, call that out explicitly instead of inventing nonexistent ROS/Gazebo hooks.

Evaluate rover response in two dimensions throughout reporting:

1. whether a real autonomy reaction occurred at all, such as obstacle avoidance, replanning, monitor-driven recovery, or another non-nominal navigation response
2. whether that reaction is attributable to baseline uncertainty, injected uncertainty, or both

If the rover performs a genuine autonomy reaction to baseline uncertainty but never reaches the injected uncertainty, report that reaction as exercised baseline autonomy and mark the injected-autonomy outcome as `UNTESTED` rather than claiming the injected challenge failed.

The same traceability discipline applies to ROS parameter consumption:

1. Identify which contract/config artifacts are plain files parsed by the scenario driver
2. Identify which values are passed as inline ROS parameters from launch
3. Ensure the contract states this mapping before implementation so launch wiring is not guessed later

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
- Require a practical reachability assessment before finalizing the injection pose or trigger. The selected trigger zone should be one the rover is likely to reach under realistic baseline behavior, not only under ideal mission geometry.
- Scale mission deadlines and success windows to the baseline route length from the launch point to the evaluated goal.
- Define how the scenario will decide whether a given behavior change was caused by baseline uncertainty, injected uncertainty, or both.
- If the scenario depends on the rover reaching a later route segment before injection, add an explicit reachability check and a fallback classification path for `UNTESTED` injected uncertainty.
- Prefer at least one trigger condition tied to observed rover behavior or mission phase, not only map geometry. Examples include entering a baseline hazard interaction zone, clearing an obstacle field, sustained forward progress, or another observable mission-phase signal.
- Define launch consumption rules up front so the scenario package does not need a separate ROS params YAML just to pass file paths or runtime settings into the driver node.


### Component Specification

Specify what the driver component should perform:

- **Monitor** the rover's execution state (position, battery, sensor status)
- **Trigger** uncertainty events at decision-critical moments (e.g., when rover commits to action)
- **Measure** adaptation success (goal completion, safety violations, contingency activations)
- **Adapt injection intensity** based on observed autonomy performance (gradually increase challenge)
- **Log behavior** for post-execution analysis (decision timestamps, fallback activations, constraint violations)
- During tuning, a small set of temporary diagnostic runtime events is recommended, for example trigger-state snapshots, reachability-state flags, or simplified phase markers. Once the scenario is stable, those events may be removed or reduced to the minimum needed for routine validation and reporting.

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
