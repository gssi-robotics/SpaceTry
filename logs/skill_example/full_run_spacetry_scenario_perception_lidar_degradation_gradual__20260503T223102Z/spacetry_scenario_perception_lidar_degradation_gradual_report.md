# Scenario Report: spacetry_scenario_perception_lidar_degradation_gradual

- schema_version: `1.0.0`
- run_label: `full_run_spacetry_scenario_perception_lidar_degradation_gradual__20260503T223102Z`
- termination_reason: `goal_reached`
- runtime_s: `308.612`
- goal_status: `PASS`
- safety_status: `DEGRADED`
- autonomy_assessment: `PASS`
- baseline_outcome_assessment: `NOT_EVALUATED`
- injected_outcome_assessment: `PASS`
- outcome_assessment: `PASS`

## Scenario Summary

This report summarizes the main autonomy outcome, runtime facts, and aggregated event statistics for the scenario run.

- primary_evaluation_target: `runtime degradation of scan-derived obstacle classification while en route to science_rock_01`
- injected_uncertainty_source: `["lidar_classification_degradation_001"]`

## Runtime Facts

- injection_status: `encountered`
- injected_uncertainty_encounter_status: `True`
- meaningful_evaluation_window_satisfied: `True`
- baseline_uncertainty_exercised_status: `False`
- baseline_confound_status: `{"block_island_near_route": true, "mr009_active": false, "mr011_active": true}`
- injected_uncertainty_source: `["lidar_classification_degradation_001"]`
- primary_observed_control_rationale: `obstacle_avoidance`
- primary_reaction_scope: `injected_only`
- primary_reaction_attribution_status: `supported`
- detection_signal_source: `/obstacle/front`

## Metrics

- autonomy_reaction_status: `True`
- autonomy_reaction_count: `1`
- adaptation_event_count: `1`
- detection_event_count: `1`
- recovery_event_count: `1`
- route_deviation_m: `19.278`
- injected_uncertainty_encounter_status: `True`
- evaluation_window_after_encounter_s: `226.5`
- meaningful_evaluation_window_satisfied: `True`
- baseline_uncertainty_exercised_status: `False`
- detection_signal_source: `/obstacle/front`
- detection_attribution_status: `True`
- minimum_fault_distance_at_detection_m: `18.185`
- baseline_confound_status: `{"block_island_near_route": true, "mr009_active": false, "mr011_active": true}`
- injected_uncertainty_source: `["lidar_classification_degradation_001"]`

## Additional Metrics

- degraded_sensing_response_latency_ms: `500.0`
- false_obstacle_rate: `1.0`
- false_obstacle_count: `677`
- missed_detection_rate: `null`
- missed_detection_count: `0`
- recovery_time_ms: `151500.0`
- post_injection_route_deviation_m: `19.278`
- injected_true_samples: `677`
- raw_hazard_samples: `0`

## Event Summaries


### Triggers

- count: `1`
- kinds: `uncertainty_injection=1`
- uncertainty_injection_time_s: `82.168`

### Detections

- count: `1`
- attribution_statuses: `supported=1`
- top_sources: `/obstacle/front=1`

### Reactions

- count: `1`
- observed_control_rationale: `obstacle_avoidance=1`
- reaction_scope: `injected_only=1`
- attribution_statuses: `supported=1`

### Adaptations

- count: `1`
- adaptation_latency_ms: `min=500.000, median=500.000, max=500.000`
- attribution_scope: `injected_only=1`
- attribution_confidence: `supported=1`
- candidate_sources: `lidar_classification_degradation_001=1`

### Recoveries

- count: `1`
- recovery_latency_ms: `min=103056.000, median=103056.000, max=103056.000`

## Safety And Goal Viability

- safety_preservation: `{"MR_009_RETURN_TO_OUTPOST_ON_LOW_BATTERY": true, "MR_011_MAINTAIN_SPEED_WHEN_FULL_BATTERY": false, "collision_with_obstacle": null}`
- goal_viability: `{"mission_deadline_met": true, "science_rock_01_reached": true}`

## Observability Notes

- No dedicated collision topic was found in the baseline stack; collision_with_obstacle is not directly observable and is reported as null.
- The scenario injects BT-facing obstacle classification messages; raw /scan is observed for attribution but not degraded directly.
- Detailed per-event records are intentionally omitted from this report; see the metrics JSON, timeline, and rosbag artifacts for raw execution data.

## Artifacts

- report: `/home/keila/robotics/marti/logs/skill_example/full_run_spacetry_scenario_perception_lidar_degradation_gradual__20260503T223102Z/spacetry_scenario_perception_lidar_degradation_gradual_report.md`
- metrics: `/home/keila/robotics/marti/logs/skill_example/full_run_spacetry_scenario_perception_lidar_degradation_gradual__20260503T223102Z/metrics/full_run_spacetry_scenario_perception_lidar_degradation_gradual__20260503T223102Z_metrics.json`
- timeline: `/home/keila/robotics/marti/logs/skill_example/full_run_spacetry_scenario_perception_lidar_degradation_gradual__20260503T223102Z/runtime/full_run_spacetry_scenario_perception_lidar_degradation_gradual__20260503T223102Z_timeline.jsonl`
- rosbags: `/home/keila/robotics/marti/logs/skill_example/full_run_spacetry_scenario_perception_lidar_degradation_gradual__20260503T223102Z/rosbags`
