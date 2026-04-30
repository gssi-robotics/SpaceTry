# Scenario Report: spacetry_scenario_navigation_obstacle_degraded_perception

- schema_version: `1.0.0`
- run_label: `full_run_shared_metrics_refactor_retry3__20260422T105138Z`
- termination_reason: `goal_reached`
- runtime_s: `280.804`
- goal_status: `PASS`
- safety_status: `FAIL`
- autonomy_assessment: `FAIL`
- baseline_outcome_assessment: `DEGRADED`
- injected_outcome_assessment: `FAIL`
- outcome_assessment: `DEGRADED`

## Scenario Summary

The inserted rock should force an avoidance maneuver while the injected obstacle-topic noise delays or destabilizes side selection and forward-progress recovery.

## Runtime Facts

- injection_status: `verified`
- injected_uncertainty_encounter_status: `True`
- meaningful_evaluation_window_satisfied: `True`
- baseline_uncertainty_exercised_status: `True`
- baseline_confound_status: `monitor_violation_active`
- injected_uncertainty_source: `["degraded_obstacle_interpretation", "runtime_route_blocking_rock"]`
- primary_observed_control_rationale: `unknown`
- primary_reaction_scope: `indeterminate`
- primary_reaction_attribution_status: `False`
- detection_signal_source: `/obstacle/left`

## Metrics

- autonomy_reaction_status: `True`
- autonomy_reaction_count: `97`
- adaptation_event_count: `39`
- detection_event_count: `880`
- recovery_event_count: `0`
- route_deviation_m: `10.412`
- injected_uncertainty_encounter_status: `True`
- evaluation_window_after_encounter_s: `91.6`
- meaningful_evaluation_window_satisfied: `True`
- baseline_uncertainty_exercised_status: `True`
- detection_signal_source: `/obstacle/left`
- detection_attribution_status: `True`
- minimum_fault_distance_at_detection_m: `11.992873249860793`
- baseline_confound_status: `monitor_violation_active`
- injected_uncertainty_source: `["degraded_obstacle_interpretation", "runtime_route_blocking_rock"]`

## Additional Metrics

- detection_rejection_counts: `{"outside_injected_attribution_window": 646}`
- false_obstacle_rate_count: `53`
- false_obstacle_rate_pct: `49.53`
- minimum_distance_to_block_island_m: `14.605`
- minimum_distance_to_injected_obstacle_m: `8.648`
- monitor_violation_count: `457`
- obstacle_detection_latency_ms: `4000.0`
- post_injection_route_deviation_m: `9.478`
- raw_scan_detection_latency_ms: `null`
- recovery_rate_ms: `null`
- total_degradation_publications: `107`

## Event Summaries


### Triggers

- count: `459`
- kinds: `baseline_monitor_violation=457, fault_encounter=1, uncertainty_injection=1`
- uncertainty_injection_time_s: `178.3`

### Detections

- count: `880`
- attribution_statuses: `rejected=646, candidate=233, supported=1`
- top_sources: `/obstacle/state=413, /obstacle/right=360, /obstacle/front=54, /obstacle/left=53`
- rejection_reasons: `outside_injected_attribution_window=646`

### Reactions

- count: `97`
- observed_control_rationale: `monitor_enforcement=96, unknown=1`
- reaction_scope: `indeterminate=58, injected_only=39`
- attribution_statuses: `unresolved=97`

### Adaptations

- count: `39`
- adaptation_latency_ms: `min=800.000, median=11708.000, max=22944.000`
- attribution_scope: `injected_only=39`
- attribution_confidence: `unresolved=39`
- candidate_sources: `degraded_obstacle_interpretation=38, runtime_route_blocking_rock=34`

## Safety And Goal Viability

- safety_preservation: `{"MR_009": true, "MR_011": false, "collision_with_obstacle": true}`
- goal_viability: `{"mission_deadline_met": true, "science_rock_01_reached": true}`

## Observability Notes

- Collision status is inferred conservatively from rover-to-fault distance because the baseline stack does not expose a direct contact signal on a ROS topic.
- The primary obstacle detection metric prefers the autonomy-facing obstacle topics, with raw /scan timing reported separately.
- Detailed per-event records are intentionally omitted from this report; see the metrics JSON, timeline, and rosbag artifacts for raw execution data.

## Artifacts

- report: `/ws/log/spacetry_scenario_navigation_obstacle_degraded_perception/full_run_shared_metrics_refactor_retry3__20260422T105138Z/spacetry_scenario_navigation_obstacle_degraded_perception_report.md`
- metrics: `/ws/log/spacetry_scenario_navigation_obstacle_degraded_perception/full_run_shared_metrics_refactor_retry3__20260422T105138Z/metrics/spacetry_scenario_navigation_obstacle_degraded_perception_metrics.json`
- timeline: `/ws/log/spacetry_scenario_navigation_obstacle_degraded_perception/full_run_shared_metrics_refactor_retry3__20260422T105138Z/runtime/spacetry_scenario_navigation_obstacle_degraded_perception_timeline.jsonl`
- rosbags: `/ws/log/spacetry_scenario_navigation_obstacle_degraded_perception/full_run_shared_metrics_refactor_retry3__20260422T105138Z/rosbags`
