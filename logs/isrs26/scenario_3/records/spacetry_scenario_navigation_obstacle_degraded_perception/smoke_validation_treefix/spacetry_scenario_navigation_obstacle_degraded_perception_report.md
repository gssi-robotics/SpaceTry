# spacetry_scenario_navigation_obstacle_degraded_perception Report

- Run label: `smoke_validation_treefix`
- Termination reason: `running`
- Outcome assessment: `UNTESTED`
- Baseline outcome assessment: `NOT_EVALUATED`
- Injected outcome assessment: `UNTESTED`

## Scenario Summary

- Primary evaluation target: runtime obstacle blocking on the route to `science_rock_01`.
- Secondary injected uncertainty: degraded obstacle interpretation on the autonomy-facing obstacle topics.
- Baseline autonomy intentionally left untouched: `src/spacetry_bt/trees/base_bt.xml`, `spacetry_bringup`, `spacetry_perception`, `spacetry_battery`, and `spacetry_monitors` remained unchanged and were only observed.

## Key Metrics

- Adaptation speed: `None` ms
- Obstacle detection latency: `None` ms via `None`
- Raw scan detection latency: `None` ms
- Recovery rate: `None` ms
- Route deviation: `4.176` m
- Post-injection route deviation: `0.0` m
- False obstacle rate: `0` / `0` (`None`%)
- Evaluation window after encounter: `0.0` s

## Safety And Goal

- MR_009 preserved: `True`
- MR_011 preserved: `False`
- Collision with injected obstacle avoided: `True`
- science_rock_01 reached: `False`
- Mission deadline met: `False`

## Attribution

- Observed control rationale: `unknown`
- Reaction scope: `indeterminate`
- Reaction attribution status: `False`
- Injected uncertainty source: `[]`
- Baseline uncertainty exercised: `False`

## Runtime Artifacts

- Metrics JSON: `/ws/log/spacetry_scenario_navigation_obstacle_degraded_perception/smoke_validation_treefix/metrics/spacetry_scenario_navigation_obstacle_degraded_perception_metrics.json`
- Timeline JSONL: `/ws/log/spacetry_scenario_navigation_obstacle_degraded_perception/smoke_validation_treefix/runtime/spacetry_scenario_navigation_obstacle_degraded_perception_timeline.jsonl`
- Rosbag directory: `/ws/log/spacetry_scenario_navigation_obstacle_degraded_perception/smoke_validation_treefix/rosbags`

## Observability Notes

- Collision status is inferred conservatively from rover-to-fault distance because the baseline stack does not expose a direct contact signal on a ROS topic.
- The primary obstacle detection metric prefers the autonomy-facing obstacle topics, with raw `/scan` timing reported separately.
