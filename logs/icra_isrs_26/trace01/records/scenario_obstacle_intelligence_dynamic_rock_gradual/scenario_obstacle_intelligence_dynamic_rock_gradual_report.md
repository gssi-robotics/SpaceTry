# Scenario Report

## Scenario
- Scenario: `scenario_obstacle_intelligence_dynamic_rock_gradual`
- Start waypoint: `dock_pad_01`
- Goal waypoint: `science_rock_01`
- Runtime obstacle pose: `(32.06, -51.20, 0.00)`
- Baseline map assessment: The active world already contains the canonical mission target science_rock_01 at approximately (50.10, -80.0) and a pre-existing hazard block_island near (20.0, -40.5). The direct dock_pad_01 to science_rock_01 segment is therefore not obstacle-free, so this scenario injects a new runtime rock farther down-route near 64 percent of the nominal dock-to-science segment to avoid trivially overlapping the baseline hazard while still blocking the preferred path toward the existing mission target.


## Outcomes
- Goal status: `FAIL`
- Safety status: `DEGRADED`
- Autonomy assessment: `FAIL`
- Finish reason: `timeout`

## Metrics
- Obstacle detection latency (ms): `800.0000000000007`
- Adaptation speed (ms): `292.0000000000016`
- Recovery rate (ms): `None`
- Detour distance (m): `0.000`
- Route deviation (m): `5.014`
- Minimum obstacle clearance (m): `28.319176636570624`
- Final battery SOC: `0.9773299694061279`

## Safety Preservation
- `MR_009_RETURN_TO_OUTPOST_ON_LOW_BATTERY`: `True`
- `MR_011_MAINTAIN_SPEED_WHEN_FULL_BATTERY`: `False`
- `collision_with_dynamic_obstacle`: `False`

## Goal Viability
- `science_rock_01_reached`: `False`
- `mission_deadline_met`: `False`

## Observability Notes
- Obstacle-detection latency is derived from the first asserted `obstacle/front|left|right` topic after injection.
- Adaptation speed is derived from the first post-injection `cmd_vel` reaction that exceeds the angular threshold or drops linear speed below the configured fraction of nominal cruise speed.
- Collision with the dynamic obstacle is inferred geometrically from rover odometry and obstacle center distance because the baseline stack does not expose a dedicated collision topic.
