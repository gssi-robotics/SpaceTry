# spacetry_scenario_navigation_obstacle_degraded_perception Report

- Run label: `full_run_20260411T100300`
- Termination reason: `goal_reached`
- Outcome assessment: `DEGRADED`
- Baseline outcome assessment: `DEGRADED`
- Injected outcome assessment: `FAIL`

## Scenario Summary

- Primary evaluation target: runtime obstacle blocking on the route to `science_rock_01`.
- Secondary injected uncertainty: degraded obstacle interpretation on the autonomy-facing obstacle topics.
- Baseline autonomy intentionally left untouched: `src/spacetry_bt/trees/base_bt.xml`, `spacetry_bringup`, `spacetry_perception`, `spacetry_battery`, and `spacetry_monitors` remained unchanged and were only observed.

## Key Metrics

- Adaptation speed: `None` ms
- Obstacle detection latency: `1900.0` ms via `/obstacle/left`
- Raw scan detection latency: `None` ms
- Recovery rate: `None` ms
- Route deviation: `10.249` m
- Post-injection route deviation: `7.817` m
- False obstacle rate: `33` / `108` (`30.56`%)
- Evaluation window after encounter: `95.2` s

## Safety And Goal

- MR_009 preserved: `True`
- MR_011 preserved: `False`
- Collision with injected obstacle avoided: `False`
- science_rock_01 reached: `True`
- Mission deadline met: `True`

## Attribution

- Observed control rationale: `unknown`
- Reaction scope: `indeterminate`
- Reaction attribution status: `False`
- Injected uncertainty source: `[]`
- Baseline uncertainty exercised: `True`

## Runtime Artifacts

- Metrics JSON: `/ws/log/spacetry_scenario_navigation_obstacle_degraded_perception/full_run_20260411T100300/metrics/spacetry_scenario_navigation_obstacle_degraded_perception_metrics.json`
- Timeline JSONL: `/ws/log/spacetry_scenario_navigation_obstacle_degraded_perception/full_run_20260411T100300/runtime/spacetry_scenario_navigation_obstacle_degraded_perception_timeline.jsonl`
- Rosbag directory: `/ws/log/spacetry_scenario_navigation_obstacle_degraded_perception/full_run_20260411T100300/rosbags`

## Observability Notes

- Collision status is inferred conservatively from rover-to-fault distance because the baseline stack does not expose a direct contact signal on a ROS topic.
- The primary obstacle detection metric prefers the autonomy-facing obstacle topics, with raw `/scan` timing reported separately.
