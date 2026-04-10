# spacetry_scenario_navigation_obstacle_sensing_coupled

- Run ID: 20260410T165009Z
- Termination reason: goal_reached
- Start wall time: 2026-04-10T16:50:10.216072+00:00
- End wall time: 2026-04-10T16:59:19.682860+00:00
- Scenario elapsed: 266.4 s

## Scenario Contract Summary

- Primary evaluation target: navigation autonomy under runtime obstacle blocking on the route to science_rock_01
- Secondary injected uncertainties: degraded obstacle interpretation on derived obstacle classification topics
- Interaction hypothesis: Degraded obstacle interpretation delays or destabilizes the rover's response to a newly inserted blocking rock and can slow or perturb recovery.
- Encounter rule: The injected uncertainty is encountered when the rover is within 11 m of the injected rock and the rock lies within +/-40 degrees of the rover heading.
- Meaningful evaluation rule: The injected-autonomy outcome is meaningfully evaluable only if evaluation_window_after_encounter_s is at least 40.0 seconds.

## Outcomes

- Goal status: PASS
- Safety status: PASS
- Autonomy assessment: UNTESTED
- Baseline outcome assessment: NOT_EVALUATED
- Injected outcome assessment: UNTESTED

## Metrics

- Adaptation speed (ms): None
- Obstacle detection latency (ms): None
- Recovery rate (ms): None
- False obstacle rate: None
- Route deviation (m): 10.1356
- Post-injection route deviation (m): None
- Evaluation window after encounter (s): 0.0
- Meaningful evaluation satisfied: False
- Observed control rationale: unknown
- Reaction scope: indeterminate
- Reaction attribution status: False
- Detection attribution status: False
- Baseline uncertainty exercised: False

## Safety Preservation

- MR_009 preserved: True
- MR_011 preserved: True
- Collision proxy preserved: True

## Goal Viability

- science_rock_01 reached: True
- Mission deadline met: True

## Traceability

- Injection entity: None
- Injection pose: None
- Active injected uncertainty sources: []
- Rosbag output dir: /ws/log/spacetry_scenario_navigation_obstacle_sensing_coupled/rosbags/20260410T165009Z

## Notes

- Baseline BT, mission config, perception node, monitors, and world file were evaluated unchanged.
- The scenario launch starts with battery=0.79 to avoid unrelated MR_011 dominance from the baseline full-speed constraint.
