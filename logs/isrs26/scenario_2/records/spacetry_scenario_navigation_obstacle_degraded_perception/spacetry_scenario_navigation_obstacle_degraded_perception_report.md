# SpaceTry Scenario Report: navigation_obstacle_degraded_perception

## Scenario Summary
- Termination reason: goal_reached
- Injection succeeded: True
- Goal status: PASS
- Safety status: FAIL
- Autonomy assessment: FAIL
- Baseline outcome assessment: PASS
- Injected outcome assessment: FAIL

## Baseline And Injected Uncertainty
- Baseline hazard exercised: True
- Injection encounter status: True
- Reaction scope: indeterminate
- Reaction attribution status: False

## Key Metrics
- Obstacle detection latency (ms): 8600.0
- Raw scan detection latency (ms): None
- Adaptation speed (ms): None
- Recovery rate (ms): 11900.0
- False obstacle rate: 0.0
- Route deviation (m): 9.913
- Post-injection route deviation (m): 9.913
- Evaluation window after encounter (s): 239.296

## Safety And Goal Viability
- Safety preservation: {"MR_009_RETURN_TO_OUTPOST_ON_LOW_BATTERY": true, "MR_011_MAINTAIN_SPEED_WHEN_FULL_BATTERY": true, "collision_with_injected_obstacle": false}
- Goal viability: {"science_rock_01_reached": true, "mission_deadline_met": true}

## Traceability Notes
- Detection signal source: /obstacle/front
- Active context at reaction: {"progress_ratio": 0.3108, "goal_distance_m": 65.104, "distance_to_injected_m": 11.505, "distance_to_baseline_hazard_m": 17.138, "battery_soc": 0.768, "battery_near_outpost": false, "obstacle_front": false, "obstacle_left": false, "obstacle_right": true, "obstacle_state": "RIGHT", "degradation_active": false, "monitor_violations": {"MR_009": false, "MR_011": false}}
- Injection gate diagnostics: {"progress_ratio": 0.0831, "distance_to_fault_m": 51.204, "recent_progress_m": 0.837, "remaining_time_s": 1170.1, "baseline_monitor_active": false, "baseline_reaction_observed": true, "clear_path_hold_s": 2.0}

## Observability Gaps
- Collision is inferred from rover-to-injected-rock distance; there is no direct baseline collision topic in the stack.
- The baseline stack does not publish /at_science_rock, so the scenario driver emits a derived status topic for monitor observability.

## Artifact Paths
- Metrics JSON: /ws/log/spacetry_scenario_navigation_obstacle_degraded_perception/metrics/scenario_metrics.json
- Runtime timeline: /ws/log/spacetry_scenario_navigation_obstacle_degraded_perception/runtime/timeline.jsonl
- Rosbags: /ws/log/spacetry_scenario_navigation_obstacle_degraded_perception/rosbags
