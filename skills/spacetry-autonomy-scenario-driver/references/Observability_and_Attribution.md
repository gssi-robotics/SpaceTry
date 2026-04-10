# Observability And Attribution

Use this reference when implementing scenario metrics, event logs, runtime timelines, rosbag capture, or final reports.

## Observability Rules

- The logging signals and events related to the scenario should be stored in a rosbag under the bind-mounted host `log/` folder and included in the final report with the metric values and scenario outcomes.
- Use ROS 2 logging and rosbag existing solutions when possible, and avoid adding new logging topics or custom solutions that are not already part of the rover's ROS 2 packages.
- If there are any gaps in observability, call them out explicitly instead of inventing nonexistent ROS/Gazebo hooks.
- Before depending on an existing SpaceTry topic for scenario triggers, derived mission state, or report metrics, confirm that the scenario node subscription QoS matches the publisher QoS used in the stack. In case of doubt, use an explicitly compatible QoS profile instead of the default subscription QoS. If there is a mismatch, treat that as an observability or integration issue to fix before evaluating the autonomy behavior.

Write scenario outputs to the bind-mounted host `log/` folder using the following per-scenario subdirectories:

- `log/scenario_<scenario_name>/scenario_<scenario_name>_report.md`
- `log/scenario_<scenario_name>/metrics/`
- `log/scenario_<scenario_name>/rosbags/`
- `log/scenario_<scenario_name>/runtime/`

## Attribution Rules

- Do not claim that the injected uncertainty was detected or that the rover reacted to it unless the report includes explicit evidence that separates the injected fault from baseline hazards, pre-existing obstacles, or unrelated monitor-triggered behavior.
- Distinguish a real rover autonomy reaction from an injected-attributed reaction.
- A real rover autonomy reaction may be credited when the rover clearly performs obstacle avoidance, replanning, or another non-nominal control response supported by logged rationale and scenario state, even if that response is caused by a baseline obstacle rather than the injected fault.
- Distinguish maneuver classification from injected-fault attribution. `observed_control_rationale` should describe what the rover did, while `reaction_attribution_status` should describe whether the injected uncertainty gets causal credit.
- If the first significant post-injection control change is labeled `goal_alignment`, `unknown`, or another non-attributable rationale, do not credit it as adaptation to the injected fault.
- A credited reaction should be supported by both event timing and logged control rationale.
- A credited detection should be supported by both event timing and scenario-specific attribution checks.
- When the autonomy logic consumes interpreted perception outputs such as BT-facing obstacle topics, detection metrics should be keyed to the first attributable autonomy-facing obstacle signal rather than delayed until a raw sensor fallback changes state.
- In multi-uncertainty scenarios, log which injected uncertainty a credited event is associated with when that is knowable, and do not imply an interaction hypothesis unless the scenario explicitly states one.

## Required Logging Content

Keep scenario logic observable:

- log injection timestamps
- log the exact fault state applied
- log why the injection was allowed at that moment
- log remaining mission time at injection
- log progress ratio at injection
- log whether any baseline monitor was already active at injection
- log the rover response signal
- write metrics that match the scenario contract
- log the injected fault pose, time, and unique identifier
- log rover pose at every credited detection and credited reaction event
- log rover-to-fault distance when detection or reaction is credited
- log which sensor, topic, or monitor signal produced each detection or reaction candidate
- log whether the credited detection came from a raw sensor path or from an autonomy-facing interpreted perception interface
- log each significant `cmd_vel` change with its command values and an explicit `observed_control_rationale` such as `goal_alignment`, `obstacle_avoidance`, `replan_execution`, `monitor_enforcement`, or `unknown`
- log the scenario state used to classify each `observed_control_rationale`, such as obstacle flags, monitor status, planner state, or BT state when available
- log `reaction_scope` for each reaction candidate as `baseline_only`, `injected_only`, `baseline_and_injected`, or `indeterminate`
- log `reaction_attribution_status` separately from `observed_control_rationale`
- log `active_context_at_reaction` as a structured snapshot of the relevant simulation properties at the reaction point
- log whether each credited detection passed the scenario's `fault_attribution_rule`
- log whether each credited reaction passed the scenario's `fault_attribution_rule`
- log the associated injected uncertainty source for each credited detection or reaction when the scenario contains more than one injected uncertainty and the source is knowable
- log the encounter timestamp and the computed `evaluation_window_after_encounter_s`
- log whether the run satisfied the `meaningful_evaluation_rule`
- log whether any baseline obstacle, monitor violation, or other confounding condition was active when a detection or reaction candidate was evaluated
- if a detection or reaction candidate is rejected, log the rejection reason

## Recommended Event Names

Prefer attribution-aware event names when scenario outputs include a runtime timeline or event log, for example:

- `fault_injected`
- `fault_injection_gate_passed`
- `fault_detection_candidate`
- `fault_detection_attributed`
- `fault_detection_rejected`
- `raw_scan_detection_attributed`
- `fault_encountered`
- `meaningful_evaluation_window_satisfied`
- `meaningful_evaluation_window_not_satisfied`
- `reaction_candidate`
- `reaction_attributed`
- `reaction_not_attributed`
- `reaction_scope_classified`
- `baseline_reaction_observed`
- `baseline_monitor_violation_active`
- `cmd_vel_changed`
- `control_rationale_classified`
