# Scenario Report: obstacle_sensing_stress

- Termination reason: `sigterm`
- Goal status: `FAIL`
- Safety status: `PASS`
- Overall injected-autonomy outcome: `INCONCLUSIVE`
- Baseline autonomy outcome: `NOT_EVALUATED`

## Scenario Summary

Delayed and noisy obstacle classifications will postpone or destabilize avoidance, increasing recovery time and route deviation after the blocker appears.


## Runtime Facts

- Injected fault verified in Gazebo: `False`
- Injected uncertainty encountered: `False`
- Meaningful evaluation rule satisfied: `False`
- Baseline uncertainty exercised: `False`
- Injected fault pose: `None`

## Metrics

- adaptation_speed_ms: `None`
- obstacle_detection_latency_ms: `None`
- raw_scan_detection_latency_ms: `None`
- recovery_rate_ms: `None`
- route_deviation_m: `1.638`
- post_injection_route_deviation_m: `0.0`
- false_obstacle_rate_pct: `0.0`
- false_obstacle_classification_count: `0`
- observed_control_rationale: `None`
- reaction_scope: `None`
- reaction_attribution_status: `False`
- evaluation_window_after_encounter_s: `0.0`

## Safety And Goal Viability

- safety_preservation: `{'MR_009': True, 'MR_011': True, 'collision_free': True}`
- goal_viability: `{'science_rock_01_reached': False, 'mission_deadline_met': False}`

## Observability Notes

- Collision outcome is based on explicit observed evidence only; this scenario does not add a new contact sensor, so collision observability remains limited.
- Detection attribution is based on geometry, runtime topic state, and baseline-hazard proximity rather than a dedicated perception provenance hook.

## Artifacts

- Metrics JSON: `/ws/log/scenario_obstacle_sensing_stress/interrupted_run_final/metrics/scenario_obstacle_sensing_stress_metrics.json`
- Runtime timeline: `/ws/log/scenario_obstacle_sensing_stress/interrupted_run_final/runtime/scenario_obstacle_sensing_stress_timeline.jsonl`
- Rosbag directory: `/ws/log/scenario_obstacle_sensing_stress/interrupted_run_final/rosbags`
