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
- `primary_evaluation_target`: the main autonomy concern or injected uncertainty the scenario is primarily intended to evaluate
- `secondary_injected_uncertainties`: zero or more additional injected uncertainties introduced in the same run
- `interaction_hypothesis`: optional statement of why multiple injected uncertainties may interact; omit when no interaction is claimed
- `injected_uncertainties`: the uncertainties introduced by the scenario driver
- `outcome_assessment`: PASS, DEGRADED, FAIL, UNTESTED, or INCONCLUSIVE for the injected-autonomy portion when the rover never reaches the injected challenge or when it is reached too late to evaluate fairly
- `baseline_outcome_assessment`: PASS, DEGRADED, FAIL, or NOT_EVALUATED for autonomy exercised by baseline uncertainties
- `injected_outcome_assessment`: PASS, DEGRADED, FAIL, UNTESTED, or INCONCLUSIVE for autonomy exercised by scenario-injected uncertainties
- `report`: the generated Markdown file under the bind-mounted host `log/` folder with the results of running the scenario, including metric values and logged signals
- `baseline_map_assessment`: a short statement of which baseline goals and obstacles already exist in the active world and how they affect route feasibility, deadline realism, and uncertainty placement
- `fault_attribution_rule`: explicit rule for deciding whether a detection or reaction is attributable to the injected uncertainty rather than baseline hazards, nominal behavior, or unrelated monitor violations
- `reaction_scope_rule`: explicit rule for deciding whether a real rover reaction counts as `baseline_only`, `injected_only`, `baseline_and_injected`, or `indeterminate`
- `control_rationale_rule`: explicit rule for how the scenario records and classifies observed control-command changes such as `goal_alignment`, `obstacle_avoidance`, `replan_execution`, `monitor_enforcement`, or `unknown`, independently from whether the injected uncertainty receives attribution credit
- `encounter_rule`: explicit rule for deciding whether the rover actually encountered the injected uncertainty or whether the run only exercised baseline uncertainty
- `meaningful_evaluation_rule`: explicit rule for deciding whether the post-encounter observation window was long enough to evaluate detection, reaction, or recovery claims fairly
- `minimum_post_encounter_observation_window_s`: minimum remaining observation time required after encounter before the injected outcome may be classified as FAIL
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

When multiple injected uncertainties are present:

- the contract should still identify one `primary_evaluation_target`
- `secondary_injected_uncertainties` may be related or unrelated to that primary target
- `interaction_hypothesis` is optional, not required

## Core Metrics

Define measurable metrics for autonomy evaluation:

- **Adaptation speed**: Time in milliseconds between uncertainty injection and the first attributable rover reaction
- **Autonomy reaction status**: Boolean indicating whether the rover performed a genuine non-nominal autonomy reaction during the run, regardless of whether it was attributable to the injected uncertainty
- **Observed control rationale**: The best classification of the observed rover maneuver at the reaction point, such as `goal_alignment`, `obstacle_avoidance`, `replan_execution`, `monitor_enforcement`, or `unknown`
- **Reaction scope**: Short string classifying whether the observed reaction is `baseline_only`, `injected_only`, `baseline_and_injected`, or `indeterminate`
- **Reaction attribution status**: Boolean indicating whether the injected uncertainty receives causal credit for the observed reaction, separate from whether the maneuver itself is classifiable
- **Active context at reaction**: Structured snapshot or key-value map of relevant runtime conditions at the reaction point, such as nearby baseline hazards, nearby injected faults, active monitors, obstacle-state signals, planner state, BT state, progress trend, or goal proximity
- **Control rationale at reaction**: Optional legacy compatibility field. If present, it must match `observed_control_rationale` and must not be used as a proxy for injected-fault attribution
- **Safety preservation**: Key-value pairs with safety constraints from monitors and boolean preservation state
- **Goal viability**: Key-value pairs with mission goal and boolean indicating goal viability
- **Recovery rate**: Time in milliseconds between rover reaction to triggered uncertainty and reaction outcome
- **Detection attribution status**: Boolean indicating whether the credited detection can be distinguished from baseline hazards, unrelated obstacle signals, or other confounding runtime events
- **Minimum fault distance at detection**: Rover-to-injected-fault distance in meters when detection is credited
- **Baseline-confound status**: Boolean or short string indicating whether a baseline obstacle, monitor violation, or unrelated runtime condition could explain the credited detection or reaction
- **Injected-uncertainty encounter status**: Boolean indicating whether the rover reached the spatial, temporal, or state conditions needed for the injected uncertainty to meaningfully affect behavior
- **Evaluation window after encounter**: Time in seconds between the first confirmed encounter with the injected uncertainty and scenario termination, timeout, or report finalization
- **Baseline-uncertainty exercised status**: Boolean indicating whether one or more baseline uncertainties materially influenced rover behavior during the run
- **Attribution scope**: Short string such as `baseline_only`, `injected_only`, `baseline_and_injected`, or `indeterminate`
- **Injected uncertainty source**: Short string or list indicating which injected uncertainty or uncertainties a credited event is associated with when that is knowable
- **Additional mission-specific metrics**: Any extra metrics requested by the user or needed for the scenario, each with an explicit unit or boolean status and a short description

Fault or injected-reaction detections must satisfy scenario-specific attribution checks such as expected sensing range, relative geometry, and consistency with the injected fault subject.

Motion reactions must not be credited from `cmd_vel` deviation alone. A credited rover reaction must be supported by a logged observed control rationale and by scenario state.

Use this distinction in the contract and report:

- a **real autonomy reaction** is a genuine non-nominal rover behavior such as obstacle avoidance, replanning, or monitor-enforced recovery
- an **injected-attributed reaction** is a real autonomy reaction that additionally satisfies the scenario's `fault_attribution_rule`

This means a baseline-obstacle avoidance event can and should count as a valid rover reaction when supported by control rationale and scenario state, even if it does not satisfy the injected-fault attribution rule.

Use the following interpretation rules:

- `observed_control_rationale` answers what maneuver the rover appears to be performing
- `reaction_scope` answers whether the maneuver is explained by baseline conditions, injected conditions, both, or cannot be separated
- `reaction_attribution_status` answers whether the injected uncertainty receives causal credit
- `active_context_at_reaction` records the simulation properties that were simultaneously relevant when the maneuver was observed

Do not use `unknown` merely because the injected uncertainty did not receive attribution credit. If the maneuver itself is classifiable, record that class in `observed_control_rationale` even when `reaction_scope` is `baseline_only` and `reaction_attribution_status` is `false`.

Use `unknown` only when the maneuver itself cannot be classified from the available evidence.

If baseline uncertainty clearly exercised autonomy but the rover never encountered the injected uncertainty, report the baseline outcome and mark the injected outcome as `UNTESTED`.

Treat `encountered` and `meaningfully evaluable` as separate concepts. A run may satisfy the `encounter_rule` but still fail the `meaningful_evaluation_rule` if too little time remains after encounter to expect attributable detection, reaction, or recovery evidence.

The report must include `evaluation_window_after_encounter_s` and compare it against `minimum_post_encounter_observation_window_s`.

An injected outcome may be classified as `FAIL` only when:

- the injected uncertainty was encountered
- the run satisfied the `meaningful_evaluation_rule`
- the remaining post-encounter observation window was reasonably long enough to expect detection, reaction, or recovery evidence

If the injected uncertainty was encountered but the remaining post-encounter observation window is too short for fair evaluation, classify the injected outcome as `INCONCLUSIVE` or `UNTESTED` according to the scenario contract instead of automatically using `FAIL`.

If attribution is ambiguous, keep the autonomy reaction credited when the behavior itself is real, but report the attribution-specific detection or reaction metric as `AMBIGUOUS` or `NOT ATTRIBUTABLE` instead of presenting it as confidently caused by the injected uncertainty.

Before execution, validate the contract against the launch design:

- every plain scenario YAML is consumed by file-path reference, not as a ROS params file
- every ROS parameter consumed by the scenario driver appears in `runtime_parameter_interface`
- every parameter listed in `runtime_parameter_interface` is passed via inline launch dict parameters
- no contract or config artifact is ambiguous about whether it is parsed by the driver or by ROS

If there is not enough information to infer any of these fields from the mission description, BT, monitors, battery, perception, and world packages, ask the user for clarification instead of making assumptions.
