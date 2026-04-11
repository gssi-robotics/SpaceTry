# Scenario Report: obstacle_sensing_stress

- Termination reason: `goal_reached`
- Goal status: `PASS`
- Safety status: `PASS`
- Overall injected-autonomy outcome: `PASS`
- Baseline autonomy outcome: `PASS`

## Scenario Summary

Delayed and noisy obstacle classifications will postpone or destabilize avoidance, increasing recovery time and route deviation after the blocker appears.


## Runtime Facts

- Injected fault verified in Gazebo: `True`
- Injected uncertainty encountered: `True`
- Meaningful evaluation rule satisfied: `True`
- Baseline uncertainty exercised: `True`
- Injected fault pose: `{'x': 41.536, 'y': -56.834, 'z': 0.0, 'yaw': -1.217}`

## Metrics

- adaptation_speed_ms: `6396.0`
- obstacle_detection_latency_ms: `8000.0`
- raw_scan_detection_latency_ms: `None`
- recovery_rate_ms: `1508.0`
- route_deviation_m: `9.746`
- post_injection_route_deviation_m: `7.458`
- false_obstacle_rate_pct: `29.67`
- false_obstacle_classification_count: `100`
- observed_control_rationale: `obstacle_avoidance`
- reaction_scope: `injected_only`
- reaction_attribution_status: `True`
- evaluation_window_after_encounter_s: `96.336`

## Safety And Goal Viability

- safety_preservation: `{'MR_009': True, 'MR_011': True, 'collision_free': True}`
- goal_viability: `{'science_rock_01_reached': True, 'mission_deadline_met': True}`

## Observability Notes

- Collision outcome is based on explicit observed evidence only; this scenario does not add a new contact sensor, so collision observability remains limited.
- Detection attribution is based on geometry, runtime topic state, and baseline-hazard proximity rather than a dedicated perception provenance hook.

## Artifacts

- Metrics JSON: `/ws/log/scenario_obstacle_sensing_stress/full_run_bt_runtime_20260411_000332/metrics/scenario_obstacle_sensing_stress_metrics.json`
- Runtime timeline: `/ws/log/scenario_obstacle_sensing_stress/full_run_bt_runtime_20260411_000332/runtime/scenario_obstacle_sensing_stress_timeline.jsonl`
- Rosbag directory: `/ws/log/scenario_obstacle_sensing_stress/full_run_bt_runtime_20260411_000332/rosbags`
