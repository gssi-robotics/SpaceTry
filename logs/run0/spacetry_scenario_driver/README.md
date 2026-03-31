# SpaceTry Scenario Driver

This scenario driver is implemented as the ROS 2 package `spacetry_scenario_driver` in `src/` and launched with a dedicated autonomy challenge launch file.

## What it does

- Relays perception outputs into the BT-facing obstacle topics
- Injects uncertainty in timed and distance-based phases during execution
- Publishes `/at_science_rock` from odometry so goal progress can be monitored
- Scores the run using goal reachability, battery margin, and monitor violations

## Main launch command

```bash
docker compose -f docker/docker-compose.yaml exec spacetry bash -lc \
  "ros2 launch spacetry_scenario_driver autonomy_challenge.launch.py headless:=0"
```

## Tunable config

Default scenario config:

- `src/spacetry_scenario_driver/config/autonomy_challenge.yaml`

You can override it at launch:

```bash
docker compose -f docker/docker-compose.yaml exec spacetry bash -lc \
  "ros2 launch spacetry_scenario_driver autonomy_challenge.launch.py \
   scenario_config:=/ws/src/SpaceTry/src/spacetry_scenario_driver/config/autonomy_challenge.yaml \
   battery:=0.45"
```

## Evaluation signals

- Raw perception: `/scenario/perception/obstacle/*`
- Final injected topics: `/obstacle/front`, `/obstacle/left`, `/obstacle/right`, `/obstacle/state`
- Goal inference: `/at_science_rock`
- Monitor verdicts: `/monitor/events`
- Scenario logging: `/scenario/events`, `/scenario/report`
