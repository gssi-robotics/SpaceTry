# Scenario Contract

Write the scenario in terms of these fields:

- `autonomy_aspect`: perception, behavior flexibility, resource awareness, mission resilience, obstacle intelligence, or safety
- `uncertainty_location`: environment, model, adaptation functions, goals, managed system, or resources
- `fault_subject`: environment object or robot subsystem
- `fault_attribute`: the targetable property that changes
- `manifestation`: stuck, noisy, or degrading
- `space_domain`: isolated, contiguous, or scattered when the fault is spatial
- `time_domain`: transient, permanent, or intermittent when the fault is temporal
- `trigger`: simulation time or robot-state predicate
- `core_metrics`: details below under **Core Metrics**
- `additional_metrics`: mission-specific metrics requested by the user or needed for the scenario
- `baseline_uncertainties`: a short inventory of uncertainties already present in the baseline SpaceTry source code, mission, world, monitors, and launch defaults that can affect the mission without any new scenario injection
- `injected_uncertainties`: the uncertainties introduced by the scenario driver
- `outcome_assessment`: PASS, DEGRADED, FAIL, or UNTESTED for the injected-autonomy portion when the rover never reaches the injected challenge
- `baseline_outcome_assessment`: PASS, DEGRADED, FAIL, or NOT_EVALUATED for autonomy exercised by baseline uncertainties
- `injected_outcome_assessment`: PASS, DEGRADED, FAIL, or UNTESTED for autonomy exercised by scenario-injected uncertainties
- `report`: the generated Markdown file under the bind-mounted host `log/` folder with the results of running the scenario, including metric values and logged signals
- `baseline_map_assessment`: a short statement of which baseline goals and obstacles already exist in the active world and how they affect route feasibility, deadline realism, and uncertainty placement
- `fault_attribution_rule`: explicit rule for deciding whether a detection or reaction is attributable to the injected uncertainty rather than baseline hazards, nominal behavior, or unrelated monitor violations
- `control_rationale_rule`: explicit rule for how the scenario records and classifies control-command changes such as `goal_alignment`, `obstacle_avoidance`, `replan_execution`, `monitor_enforcement`, or `unknown`
- `encounter_rule`: explicit rule for deciding whether the rover actually encountered the injected uncertainty or whether the run only exercised baseline uncertainty
- `runtime_parameter_interface`: explicit list of ROS node parameters passed at launch time as inline launch-dict values, including parameter name, type, meaning, and the contract field or file path that feeds each one
- `artifact_consumption_rule`: explicit statement of how each scenario artifact is consumed by ROS, launch, and the driver node without ambiguity

## ROS Consumption By Design

The contract must define not only what the scenario means, but also how each artifact is consumed at runtime.

Use the following design-time rules:

- Scenario contract YAML and scenario config YAML are plain scenario artifacts, not ROS parameter files.
- Scenario nodes should receive ROS parameters only through inline launch dict parameters.
- Do not introduce a separate `driver_params.yaml` design path for scenario drivers in this skill.
- If a node needs access to a scenario contract or scenario config file, pass the file path as a string ROS parameter such as `scenario_contract_file` or `scenario_config_file`.
- Only YAML files with a proper ROS parameter structure rooted in `<node_name>: ros__parameters:` may ever be passed to `Node(parameters=[...])` as files.
- If a scenario does not intentionally create such a ROS-native params file, then `Node(parameters=[...])` should contain only inline dictionaries.

The `artifact_consumption_rule` must at minimum distinguish:

- which artifacts are parsed by the scenario driver as plain YAML files
- which values are passed to ROS as inline launch parameters
- which values are launch arguments only
- which files must never be treated as ROS parameter files

The `runtime_parameter_interface` must at minimum include:

- `parameter_name`
- `type`
- `consumer_node`
- `source`
- `transport`

where `transport` should be `inline_launch_dict` for scenario-driver ROS parameters in this skill.

## Core Metrics

Define measurable metrics for autonomy evaluation:

- **Adaptation speed**: Time in milliseconds between uncertainty injection and the first attributable rover reaction
- **Reaction attribution status**: Boolean indicating whether the credited reaction can be distinguished from nominal behavior, baseline hazards, or unrelated monitor-triggered behavior
- **Control rationale at reaction**: The logged rationale attached to the credited control response, such as `goal_alignment`, `obstacle_avoidance`, `replan_execution`, `monitor_enforcement`, or `unknown`
- **Safety preservation**: Key-value pairs with safety constraints from monitors and boolean preservation state
- **Goal viability**: Key-value pairs with mission goal and boolean indicating goal viability
- **Recovery rate**: Time in milliseconds between rover reaction to triggered uncertainty and reaction outcome
- **Detection attribution status**: Boolean indicating whether the credited detection can be distinguished from baseline hazards, unrelated obstacle signals, or other confounding runtime events
- **Minimum fault distance at detection**: Rover-to-injected-fault distance in meters when detection is credited
- **Baseline-confound status**: Boolean or short string indicating whether a baseline obstacle, monitor violation, or unrelated runtime condition could explain the credited detection or reaction
- **Injected-uncertainty encounter status**: Boolean indicating whether the rover reached the spatial, temporal, or state conditions needed for the injected uncertainty to meaningfully affect behavior
- **Baseline-uncertainty exercised status**: Boolean indicating whether one or more baseline uncertainties materially influenced rover behavior during the run
- **Attribution scope**: Short string such as `baseline_only`, `injected_only`, `baseline_and_injected`, or `indeterminate`
- **Additional mission-specific metrics**: Any extra metrics requested by the user or needed for the scenario, each with an explicit unit or boolean status and a short description

Fault or reaction detections must satisfy scenario-specific attribution checks such as expected sensing range, relative geometry, and consistency with the injected fault subject.

Motion reactions must not be credited from `cmd_vel` deviation alone. A credited rover reaction must be supported by a logged control rationale and by scenario state that distinguishes injected-fault response from nominal goal alignment, baseline hazards, monitor enforcement, or unrelated runtime behavior.

If baseline uncertainty clearly exercised autonomy but the rover never encountered the injected uncertainty, report the baseline outcome and mark the injected outcome as `UNTESTED`.

If attribution is ambiguous, report the relevant detection or reaction metric as `AMBIGUOUS` or `NOT ATTRIBUTABLE` instead of presenting a numeric value as though it were confidently caused by the injected uncertainty.

Before execution, validate the contract against the launch design:

- every plain scenario YAML is consumed by file-path reference, not as a ROS params file
- every ROS parameter consumed by the scenario driver appears in `runtime_parameter_interface`
- every parameter listed in `runtime_parameter_interface` is passed via inline launch dict parameters
- no contract or config artifact is ambiguous about whether it is parsed by the driver or by ROS

If there is not enough information to infer any of these fields from the mission description, BT, monitors, battery, perception, and world packages, ask the user for clarification instead of making assumptions.
