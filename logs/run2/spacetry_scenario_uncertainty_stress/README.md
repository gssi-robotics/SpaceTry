# SpaceTry Scenario: Uncertainty Stress

This package adds a standalone scenario driver that evaluates the baseline rover autonomy without modifying the behavior tree, perception, battery, or monitor packages under test.

## Scenario contract

- `autonomy_aspect`: perception, resource awareness, behavior flexibility, safety
- `uncertainty_location`: adaptation functions, environment, resources
- `uncertainty_nature`: variability and epistemic
- `measurements`: adaptation speed, recovery rate, safety preservation, goal viability
- `success_outcome`:
  - `achieved`: adapted and preserved safety constraints
  - `degraded`: adapted but one or more safety constraints were violated
  - `failed`: no meaningful adaptation observed before timeout

## Injection plan

1. `decision_point_perception_ramp`
   - Timing: decision point proxy (`/cmd_vel` begins the navigation commitment)
   - Intensity: gradual
   - Fault subject: robot sensor
   - Fault attribute: obstacle perception output
   - Manifestation: degrading
2. `mid_action_battery_drop`
   - Timing: mid-action while the rover is travelling away from the outpost
   - Intensity: sudden
   - Fault subject: robot battery
   - Fault attribute: energy level
   - Manifestation: stuck
3. `contingency_obstacle_cascade`
   - Timing: during the low-battery contingency window
   - Intensity: cascading
   - Fault subject: environment hazards
   - Fault attribute: traversability / obstacle availability
   - Manifestation: intermittent

## Traceability notes

- The scenario uses topic-level overrides on `/battery/soc`, `/battery/near_outpost`, and `obstacle/*` as the least-invasive injection hooks.
- A direct Gazebo obstacle insertion hook is not currently available in this package, so dynamic hazards are emulated at the rover perception interface.
- Behavior-tree decision points are approximated from rover motion (`/cmd_vel`) because the baseline BT does not expose explicit decision-state topics.

## Reports

By default the driver writes timestamped JSON reports to `/tmp/spacetry_scenario_uncertainty_stress`.

Run with:

```bash
docker compose -f docker/docker-compose.yaml exec spacetry bash -lc \
  "source /opt/ros/spaceros/setup.bash && source /ws/install/setup.bash && \
   ros2 launch spacetry_scenario_uncertainty_stress scenario_uncertainty_stress.launch.py"
```
