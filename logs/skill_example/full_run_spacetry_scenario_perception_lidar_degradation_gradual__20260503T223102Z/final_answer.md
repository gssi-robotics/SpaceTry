Implemented and executed the autonomy test scenario driver.

Created new ROS 2 scenario package:
- [driver.py](/home/keila/robotics/marti/logs/skill_example/spacetry_scenario_perception_lidar_degradation_gradual/spacetry_scenario_perception_lidar_degradation_gradual/driver.py)
- [scenario_config.yaml](/home/keila/robotics/marti/logs/skill_example/spacetry_scenario_perception_lidar_degradation_gradual/config/scenario_config.yaml)
- [scenario_contract.yaml](/home/keila/robotics/marti/logs/skill_example/spacetry_scenario_perception_lidar_degradation_gradual/config/scenario_contract.yaml)
- [launch file](/home/keila/robotics/marti/logs/skill_example/spacetry_scenario_perception_lidar_degradation_gradual/launch/scenario_perception_lidar_degradation_gradual.launch.py)

The driver injects runtime degradation into BT-facing scan-derived obstacle classification by publishing intermittent/degrading false obstacle pulses on `/obstacle/front`, `/obstacle/left`, and `/obstacle/right`, while observing raw `/scan` for attribution. I left the baseline BT, perception, monitors, mission, and world packages untouched.

Execution completed through Docker:
- Rebuilt stale `spacetry:dev` image.
- Started/recreated the `spacetry` container.
- Copied runtime packages into `/ws/src`.
- Rebuilt affected packages inside the container.
- Ran scenario preflight successfully.
- Ran interrupted validation successfully.
- Ran full scenario via `run_scenario_full.sh`.

Final full-run result:
- `launch_status`: `0`
- `termination_reason`: `goal_reached`
- `main_result_eligible`: `yes`
- `goal_status`: `PASS`
- `safety_status`: `DEGRADED`
- `autonomy_assessment`: `PASS`
- `outcome_assessment`: `PASS`

Key metrics:
- `science_rock_01_reached`: `true`
- `mission_deadline_met`: `true`
- `MR_009_RETURN_TO_OUTPOST_ON_LOW_BATTERY`: `true`
- `MR_011_MAINTAIN_SPEED_WHEN_FULL_BATTERY`: `false`
- `degraded_sensing_response_latency_ms`: `500.0`
- `false_obstacle_rate`: `1.0`
- `false_obstacle_count`: `677`
- `recovery_time_ms`: `151500.0`
- `route_deviation_m`: `19.278`
- `post_injection_route_deviation_m`: `19.278`

Artifacts:
- Report: [spacetry_scenario_perception_lidar_degradation_gradual_report.md](/home/keila/robotics/marti/logs/skill_example/full_run_spacetry_scenario_perception_lidar_degradation_gradual__20260503T223102Z/spacetry_scenario_perception_lidar_degradation_gradual_report.md)
- Wrapper summary: [wrapper_run_summary.txt](/home/keila/robotics/marti/logs/skill_example/full_run_spacetry_scenario_perception_lidar_degradation_gradual__20260503T223102Z/runtime/wrapper_run_summary.txt)
- Metrics JSON: [metrics](/home/keila/robotics/marti/logs/skill_example/full_run_spacetry_scenario_perception_lidar_degradation_gradual__20260503T223102Z/metrics/full_run_spacetry_scenario_perception_lidar_degradation_gradual__20260503T223102Z_metrics.json)

One important interpretation note: the run achieved the mission and recovered from injected perception degradation, but safety is marked `DEGRADED` because MR_011 fired repeatedly while the battery was full and commanded speed was below the full-speed threshold. Collision is reported as `null` because the baseline stack does not expose a dedicated collision topic.
