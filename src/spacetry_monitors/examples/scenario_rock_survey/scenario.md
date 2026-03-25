# Scenario: Rock Survey at Ridge

## Natural-language mission description

The Curiosity rover must navigate from its dock pad to the ridge waypoint,
survey the area for science rocks, and return to the outpost habitat.
During the entire mission the rover must avoid collisions with obstacles.
The rover must not exceed a safe velocity.  If the rover detects an obstacle
within the safety threshold it must stop immediately and must not resume
forward motion until the obstacle is no longer within the threshold.
The rover must eventually reach the ridge waypoint and must eventually
return to the outpost habitat.

## Environment

- Mars outpost terrain with scattered rocks and one known hazard
  (block_island meteorite).
- Obstacle detection via forward-facing lidar at `/scan`.
- Odometry from `/model/curiosity_mars_rover/odometry`.
- Waypoints: ridge (12, 4) and outpost_habitat_01 (145, 50).
- No battery model exists — energy exhaustion is not monitored.

## Derived minimal safety rules (plain English)

1. **Obstacle stop** (guarantee): When an obstacle is detected within 1.0 m,
   the rover must immediately stop.
2. **No forward while obstacle** (guarantee): While an obstacle is within
   1.0 m, the rover must not command forward motion.
3. **Velocity cap** (guarantee): The rover's forward velocity must never
   exceed 1.5 m/s.
4. **Environment clearance** (assumption): The environment always has at
   least some path with obstacle_min_range > 0 — i.e., the rover is not
   fully surrounded with no escape.
5. **Mission liveness — reach ridge** (guarantee): The rover must eventually
   satisfy at_waypoint when the target is the ridge.
6. **Mission liveness — return home** (guarantee): The rover must eventually
   satisfy at_waypoint when the target is the outpost habitat.

## Rationale for exclusions

- No battery requirement: battery_percentage is not implemented.
- No sample-collection requirement: sample_detected is a random stub;
  requiring successful sampling would not be monitorable.
- No specific turn direction or exploration pattern: those are BT tactics.
- No mast/arm deployment timing: those are convenience, not safety.
