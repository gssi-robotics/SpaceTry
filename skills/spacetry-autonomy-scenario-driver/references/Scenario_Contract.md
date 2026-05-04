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
- `trigger_events`: recorded moments where adaptation may be requested, including injected uncertainties and relevant baseline triggers
- `detection_events`: recorded detection candidates or attributed detections, including their timing and evidence references
- `reaction_events`: recorded non-nominal rover reactions, including rationale, scope, attribution status, and active context
- `recovery_events`: recorded recovery or stable-progress observations linked to one or more reactions
- `adaptation_events`: recorded trigger-to-reaction pairings with `adaptation_latency_ms` and attribution metadata
- `summary_metrics`: normalized stable run-level metrics for comparison across scenarios
- `additional_metrics`: scenario-specific extras, user-requested metrics, or any new metric needed when it is not already covered by the normalized `summary_metrics` schema
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
- `monitor_handling`: internal normalized structure for the monitors that can affect trigger gating, attribution logic, or report interpretation for the scenario
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

## Monitor Handling

The internal contract must explicitly say how the scenario uses monitor topics at runtime.

The `monitor_handling` structure is an internal planning artifact for reproducibility and attribution. It should be derived from the user prompt's free-form monitor notes, plus any monitor behavior discovered while inspecting the monitors package, launch wiring, or runtime evidence.

Use `monitor_handling` only for monitors that are material to the scenario design or to interpreting the run. Do not turn it into a full catalog of all baseline monitors unless they genuinely affect the scenario.

Each entry should include at minimum:

- `monitor_name`
- `topic`
- `consumer_node`
- `affects_trigger_gating`
- `affects_attribution`
- `affects_report_interpretation`
- `rationale`

Use explicit booleans for the three `affects_*` fields rather than a single enum.

If a monitor is a plausible confound or was reviewed during planning but is intentionally not consumed by the scenario, you may still keep an internal entry with all `affects_*` fields set to `false` and a short rationale. Do not surface those non-influential entries in the final report unless they actually affected runtime interpretation.

When multiple injected uncertainties are present:

- the contract should still identify one `primary_evaluation_target`
- `secondary_injected_uncertainties` may be related or unrelated to that primary target
- `interaction_hypothesis` is optional, not required

## Event Collections And Summary Metrics

Use an event-centric data model for autonomy evaluation.

The scenario driver should record:

- **Trigger events**: Every moment where adaptation may be requested, including runtime injections and any baseline trigger that materially affects interpretation
- **Detection events**: Detection candidates or attributed detections with timing, linked trigger IDs, evidence references, and optional minimum fault distance
- **Reaction events**: Every recorded non-nominal rover reaction, with `observed_control_rationale`, `reaction_scope`, `attribution_status`, `active_context_at_reaction`, `candidate_sources`, and `evidence_refs`
- **Recovery events**: Recovery or stable-progress observations linked to one or more reactions
- **Adaptation events**: Explicit trigger-to-reaction pairings. Each adaptation event must include `trigger_id`, `reaction_id`, `adaptation_latency_ms`, attribution metadata, candidate sources, and evidence references

The `summary_metrics` section should keep only normalized stable run-level values meant to stay comparable across scenarios, such as:

- **Autonomy reaction status**: Boolean indicating whether the rover performed one or more genuine non-nominal reactions during the run
- **Autonomy reaction count**: Count of recorded reaction events
- **Adaptation event count**: Count of recorded adaptation events
- **Detection event count**: Count of recorded detection events
- **Recovery event count**: Count of recorded recovery events
- **Detection signal source**: Topic, sensor, or interface that produced the credited detection summary, for example `/obstacle/state`, `/obstacle/front`, or `/scan`
- **Safety preservation**: Key-value pairs with safety constraints from monitors and boolean preservation state
- **Goal viability**: Key-value pairs with mission goal and boolean indicating goal viability
- **Route deviation**: Maximum or average lateral deviation between the full executed route and the nominal route over the whole run. If the scenario also needs disturbance-scoped deviation, report it separately as an additional metric such as `post_injection_route_deviation_m`
- **Detection attribution status**: Boolean indicating whether the credited detection summary can be distinguished from baseline hazards, unrelated obstacle signals, or other confounding runtime events
- **Minimum fault distance at detection**: Rover-to-injected-fault distance in meters when a credited detection summary is available
- **Baseline-confound status**: Boolean or short string indicating whether a baseline obstacle, monitor violation, or unrelated runtime condition could explain the credited detection or reaction
- **Injected-uncertainty encounter status**: Boolean indicating whether the rover reached the spatial, temporal, or state conditions needed for the injected uncertainty to meaningfully affect behavior
- **Evaluation window after encounter**: Time in seconds between the first confirmed encounter with the injected uncertainty and scenario termination, timeout, or report finalization
- **Baseline-uncertainty exercised status**: Boolean indicating whether one or more baseline uncertainties materially influenced rover behavior during the run
- **Injected uncertainty source**: Short string or list indicating which injected uncertainty or uncertainties were materially involved in the recorded adaptation events when that is knowable
- **Do not add ad hoc prompt-specific metrics here**: If a metric was requested by the user or introduced by scenario design and it is not already part of the normalized `summary_metrics` schema, place it under `additional_metrics`

Use `additional_metrics` for:

- user-requested metrics from the prompt when they are not already represented by an existing normalized `summary_metrics` field
- scenario-specific extras that are useful for interpretation but are not part of the shared comparison schema
- any new metric introduced during development for a specific uncertainty family or observability need

Do not duplicate a metric in both sections just because the user requested it. If a requested metric already matches a normalized `summary_metrics` field such as `safety_preservation`, `goal_viability`, or `route_deviation_m`, satisfy the request through `summary_metrics` and use `additional_metrics` only for the remaining non-canonical items.

Do not collapse all adaptation behavior into a single run-level `adaptation_speed_ms`. Record every adaptation event separately with `adaptation_latency_ms`.

Fault or injected-reaction detections must satisfy scenario-specific attribution checks such as expected sensing range, relative geometry, and consistency with the injected fault subject.

If the autonomy stack consumes interpreted obstacle topics or other processed perception signals, a credited obstacle detection may come from those autonomy-facing signals even when a raw sensor fallback such as `/scan` still appears `CLEAR`. Do not force `obstacle_detection_latency_ms` to depend on raw-sensor evidence unless raw sensing is itself the autonomy interface under evaluation.

If raw-sensor timing is also analytically useful, report it as a separate additional metric such as `raw_scan_detection_latency_ms` instead of overloading the core obstacle-detection metric.

Motion reactions must not be credited from `cmd_vel` deviation alone. A recorded reaction event must be supported by a logged observed control rationale and by scenario state.

Use this distinction in the contract and report:

- a **real autonomy reaction** is a genuine non-nominal rover behavior such as obstacle avoidance, replanning, or monitor-enforced recovery
- an **injected-attributed reaction** is a real autonomy reaction that additionally satisfies the scenario's `fault_attribution_rule`

This means a baseline-obstacle avoidance event can and should count as a valid rover reaction when supported by control rationale and scenario state, even if it does not satisfy the injected-fault attribution rule.

Use the following interpretation rules:

- `observed_control_rationale` answers what maneuver the rover appears to be performing
- `reaction_scope` answers whether the maneuver is explained by baseline conditions, injected conditions, both, or cannot be separated
- `reaction_attribution_status` answers whether the injected uncertainty receives causal credit
- `active_context_at_reaction` records the simulation properties that were simultaneously relevant when the maneuver was observed
- `adaptation_latency_ms` belongs to the `adaptation_events` collection, not to a single run-level scalar

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
- every monitor that can affect trigger gating, attribution, or report interpretation appears in `monitor_handling`
- each `monitor_handling` entry uses explicit `affects_*` fields plus a rationale instead of enum shorthand
- event collections and `summary_metrics` follow the shared schema provided by `src/spacetry_scenario_metrics/`
- the final report and runtime logs mention only the monitors that actually influenced trigger gating, attribution, or result interpretation during the run

If there is not enough information to infer any of these fields from the mission description, BT, monitors, battery, perception, and world packages, ask the user for clarification instead of making assumptions.
