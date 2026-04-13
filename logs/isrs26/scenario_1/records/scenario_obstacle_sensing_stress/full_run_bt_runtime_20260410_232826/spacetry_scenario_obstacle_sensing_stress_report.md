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
- Injected fault pose: `{'x': 41.501, 'y': -56.466, 'z': 0.0, 'yaw': -1.22}`

## Metrics

- adaptation_speed_ms: `6364.0`
- obstacle_detection_latency_ms: `8000.0`
- raw_scan_detection_latency_ms: `None`
- recovery_rate_ms: `3044.0`
- route_deviation_m: `10.039`
- post_injection_route_deviation_m: `7.643`
- false_obstacle_rate_pct: `43.45`
- false_obstacle_classification_count: `146`
- observed_control_rationale: `obstacle_avoidance`
- reaction_scope: `injected_only`
- reaction_attribution_status: `True`
- evaluation_window_after_encounter_s: `97.796`

## Safety And Goal Viability

- safety_preservation: `{'MR_009': True, 'MR_011': True, 'collision_free': True}`
- goal_viability: `{'science_rock_01_reached': True, 'mission_deadline_met': True}`

## Observability Notes

- Collision outcome is based on explicit observed evidence only; this scenario does not add a new contact sensor, so collision observability remains limited.
- Detection attribution is based on geometry, runtime topic state, and baseline-hazard proximity rather than a dedicated perception provenance hook.

## Artifacts

- Metrics JSON: `/ws/log/scenario_obstacle_sensing_stress/full_run_bt_runtime_20260410_232826/metrics/scenario_obstacle_sensing_stress_metrics.json`
- Runtime timeline: `/ws/log/scenario_obstacle_sensing_stress/full_run_bt_runtime_20260410_232826/runtime/scenario_obstacle_sensing_stress_timeline.jsonl`
- Rosbag directory: `/ws/log/scenario_obstacle_sensing_stress/full_run_bt_runtime_20260410_232826/rosbags`
