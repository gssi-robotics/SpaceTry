# spacetry_scenario_navigation_obstacle_sensing_coupled

- Run ID: 20260410T054613Z
- Termination reason: signal_sigint
- Start wall time: 2026-04-10T05:46:15.212912+00:00
- End wall time: 2026-04-10T05:50:33.607984+00:00
- Scenario elapsed: 110.0 s

## Scenario Contract Summary

- Primary evaluation target: navigation autonomy under runtime obstacle blocking on the route to science_rock_01
- Secondary injected uncertainties: degraded obstacle interpretation on derived obstacle classification topics
- Interaction hypothesis: Degraded obstacle interpretation delays or destabilizes the rover's response to a newly inserted blocking rock and can slow or perturb recovery.
- Encounter rule: The injected uncertainty is encountered when the rover is within 11 m of the injected rock and the rock lies within +/-40 degrees of the rover heading.
- Meaningful evaluation rule: The injected-autonomy outcome is meaningfully evaluable only if evaluation_window_after_encounter_s is at least 40.0 seconds.

## Outcomes

- Goal status: FAIL
- Safety status: PASS
- Autonomy assessment: DEGRADED
- Baseline outcome assessment: NOT_EVALUATED
- Injected outcome assessment: DEGRADED

## Metrics

- Adaptation speed (ms): 700.0
- Obstacle detection latency (ms): None
- Recovery rate (ms): None
- False obstacle rate: 0.4059
- Route deviation (m): 6.7318
- Evaluation window after encounter (s): 56.8
- Meaningful evaluation satisfied: True
- Observed control rationale: obstacle_avoidance
- Reaction scope: injected_only
- Reaction attribution status: True
- Detection attribution status: False
- Baseline uncertainty exercised: False

## Safety Preservation

- MR_009 preserved: True
- MR_011 preserved: True
- Collision proxy preserved: True

## Goal Viability

- science_rock_01 reached: False
- Mission deadline met: False

## Traceability

- Injection entity: injected_blocking_rock_20260410t054613z
- Injection pose: {'x': 16.7225, 'y': -20.2034, 'z': -1.2, 'yaw': -1.0617}
- Active injected uncertainty sources: ['runtime_blocking_rock', 'degraded_obstacle_interpretation']
- Rosbag output dir: /ws/log/spacetry_scenario_navigation_obstacle_sensing_coupled/rosbags/20260410T054613Z

## Notes

- Baseline BT, mission config, perception node, monitors, and world file were evaluated unchanged.
- The scenario launch starts with battery=0.79 to avoid unrelated MR_011 dominance from the baseline full-speed constraint.
