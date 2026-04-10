# spacetry_scenario_navigation_obstacle_sensing_coupled

- Run ID: 20260410T171203Z
- Termination reason: goal_reached
- Start wall time: 2026-04-10T17:12:04.555565+00:00
- End wall time: 2026-04-10T17:21:41.398700+00:00
- Scenario elapsed: 272.2 s

## Scenario Contract Summary

- Primary evaluation target: navigation autonomy under runtime obstacle blocking on the route to science_rock_01
- Secondary injected uncertainties: degraded obstacle interpretation on derived obstacle classification topics
- Interaction hypothesis: Degraded obstacle interpretation delays or destabilizes the rover's response to a newly inserted blocking rock and can slow or perturb recovery.
- Encounter rule: The injected uncertainty is encountered when the rover is within 11 m of the injected rock and the rock lies within +/-40 degrees of the rover heading.
- Meaningful evaluation rule: The injected-autonomy outcome is meaningfully evaluable only if evaluation_window_after_encounter_s is at least 40.0 seconds.

## Outcomes

- Goal status: PASS
- Safety status: FAIL
- Autonomy assessment: FAIL
- Baseline outcome assessment: NOT_EVALUATED
- Injected outcome assessment: FAIL

## Metrics

- Adaptation speed (ms): 660.0
- Obstacle detection latency (ms): None
- Recovery rate (ms): None
- False obstacle rate: 0.8054
- Route deviation (m): 10.3534
- Post-injection route deviation (m): 10.3534
- Evaluation window after encounter (s): 223.5
- Meaningful evaluation satisfied: True
- Observed control rationale: obstacle_avoidance
- Reaction scope: injected_only
- Reaction attribution status: True
- Detection attribution status: False
- Baseline uncertainty exercised: False

## Safety Preservation

- MR_009 preserved: True
- MR_011 preserved: True
- Collision proxy preserved: False

## Goal Viability

- science_rock_01 reached: True
- Mission deadline met: True

## Traceability

- Injection entity: injected_blocking_rock_20260410t171203z
- Injection pose: {'x': 15.9091, 'y': -20.7311, 'z': -1.2, 'yaw': -1.0476}
- Active injected uncertainty sources: ['runtime_blocking_rock', 'degraded_obstacle_interpretation']
- Rosbag output dir: /ws/log/spacetry_scenario_navigation_obstacle_sensing_coupled/rosbags/20260410T171203Z

## Notes

- Baseline BT, mission config, perception node, monitors, and world file were evaluated unchanged.
- The scenario launch starts with battery=0.79 to avoid unrelated MR_011 dominance from the baseline full-speed constraint.
