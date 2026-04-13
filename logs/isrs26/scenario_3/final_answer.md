The full scenario ran uninterrupted and completed naturally before the 1200 s timeout. The BT-driven mission ended with `termination_reason = goal_reached` at about `281.8 s`, and there are no scenario processes still running.

Key outcomes:
- Goal: `science_rock_01_reached=true`, `mission_deadline_met=true`
- Baseline safety: `MR_009=true`, `MR_011=false`, so baseline outcome is `DEGRADED`
- Injected uncertainty: rock spawn was verified at `179.5 s`, encounter happened at `186.5 s`, detection latency was `1900 ms`, evaluation window after encounter was `95.2 s`
- Final assessments: `baseline_outcome_assessment=DEGRADED`, `injected_outcome_assessment=FAIL`, `outcome_assessment=DEGRADED`

Artifacts are here:
- [report.md](log/spacetry_scenario_navigation_obstacle_degraded_perception/full_run_20260411T100300/spacetry_scenario_navigation_obstacle_degraded_perception_report.md)
- [metrics.json](log/spacetry_scenario_navigation_obstacle_degraded_perception/full_run_20260411T100300/metrics/spacetry_scenario_navigation_obstacle_degraded_perception_metrics.json)
- [timeline.jsonl](log/spacetry_scenario_navigation_obstacle_degraded_perception/full_run_20260411T100300/runtime/spacetry_scenario_navigation_obstacle_degraded_perception_timeline.jsonl)
- [rosbag.mcap](log/spacetry_scenario_navigation_obstacle_degraded_perception/full_run_20260411T100300/rosbags/spacetry_scenario_navigation_obstacle_degraded_perception_bag/spacetry_scenario_navigation_obstacle_degraded_perception_bag_0.mcap)

One caveat: the Markdown report’s collision wording is inconsistent with the raw metric field. The raw metrics say `collision_with_obstacle=false`, so I’d trust the JSON over that sentence in the report.
