# Space Robotics Use Case Description
## RoME Working Session 5 Mission Description
The idea is that you design a mission that explores an unknown environment surrounding the rover to find sampling locations when a payload is being triggered by a science instrument. In this case, the camera and the lidar. Figure 1 below depicts the overall idea of the mission, where the exploration goal is to find areas that align with the robot’s sampling and sensing capabilities.
![Intersection between the unknown and the known environments during robot's operation.](robotics_ops.png)
<p style="text-align: center;"> Fig. 1: Robotics mission goals.</p>

### Goals
The main goal of the deployed rovers is to conduct scientific exploration and sampling of Mars. Depending on the rover payload configuration, different types of samples  or payload data can be collected. For the NASA Curiosity rover a detailed list of instruments is provided [here](https://science.nasa.gov/resource/diverse-science-payload-on-mars-rover-curiosity/). 


The goal of the rover mission will be to explore the surrounding environment starting from the dock_pad location. To increase autonomy, the mission goal is to find a rock sample. An important aspect in the design to consider are the <u>terrain obstacles</u> and <u>battery energy levels</u>.

### Higher-Level Rover Tasks
1. **Explore**
    - Navigate safely towards unknown areas (not explored yet)
    - Circumvent obstacles detected by the LiDAR
2. **Sampling**
    - Identify (with camera) and count samples
3. **Recharge**
    - Battery levels should be considered to determine the next task


## NASA Curiosity Rover Model in Space-ROS

The Curiosity rover model comes from the [Space ROS Demos](https://github.com/space-ros/demos/tree/main/curiosity_rover) repository and is imported via `vcs` using `deps/spacetry.repos` (pinned at commit `acf369b`). The import brings three ROS 2 packages:

| Package | Role |
|---|---|
| `curiosity_description` | URDF/xacro model of the rover (meshes, joints, sensors, ros2_control config) |
| `curiosity_gazebo` | Gazebo models (terrain, paths), plugins, and the `odom_tf_publisher` helper node |
| `curiosity_rover_demo` | Demo nodes for arm, mast, wheel control and the `mars_rover.launch.py` integration launch |

At launch, `spacetry_bringup` processes the xacro (`curiosity_mars_rover.xacro`) into a URDF XML and spawns the rover into the `mars_outpost` world via `ros_gz_sim create`. After spawn, the launch file loads a controller chain through `ros2_control`:

1. `GazeboSystem` hardware interface → active
2. `joint_state_broadcaster` → active
3. In parallel: `arm_joint_trajectory_controller`, `mast_joint_trajectory_controller`, `wheel_velocity_controller`, `steer_position_controller`, `wheel_tree_position_controller`

The demo nodes from `curiosity_rover_demo` (`mars_rover.launch.py`) expose ROS 2 services for rover actions:
`/move_forward`, `/move_stop`, `/turn_left`, `/turn_right`, `/open_arm`, `/close_arm`, `/mast_open`, `/mast_close`, `/mast_rotate`

### Rover Sensor Topics

| Sensor | ROS 2 Topic | Message Type | Notes |
|---|---|---|---|
| Camera | `/image_raw` | `sensor_msgs/Image` | Bridged via `ros_gz_image` |
| LiDAR | `/scan` | `sensor_msgs/LaserScan` | Bridged via `ros_gz_bridge` |
| Odometry | `/model/curiosity_mars_rover/odometry` | `nav_msgs/Odometry` | Bridged via `ros_gz_bridge` |
| Joint states | `/joint_states` | `sensor_msgs/JointState` | Published by `joint_state_broadcaster` |
| Clock | `/clock` | `rosgraph_msgs/Clock` | Simulation time; all nodes use `use_sim_time: true` |

## Mars Outpost World

The Gazebo world (`mars_outpost.sdf`) defines a Martian environment with reduced gravity (3.711 m/s²) and the following scene entities:

| Instance name | Model | Role |
|---|---|---|
| `outpost_habitat_01` | `station` | Outpost base station |
| `solar_panel` | `solar_panel` | Power infrastructure (static) |
| `science_rock_01` | `rock_5` | Science sampling target |
| `block_island` | `block_island` | Terrain hazard obstacle |
| `curiosity_path` | `curiosity_path` | Ground terrain (from `curiosity_gazebo`) |

Physics is tuned for performance: 4 ms step size at 250 Hz update rate (ODE solver, `quick` type), with shadows disabled.

## SpaceTry Packages and Their Mission Role

### `spacetry_bt` — Behavior Tree mission runner

The BT runner (`spacetry_bt_runner`) loads a BehaviorTree.CPP XML file and ticks it at a configurable rate (default 10 Hz). It provides 6 custom BT nodes:

| BT Node | Type | Description |
|---|---|---|
| `CallEmptyService` | SyncAction | Calls any `std_srvs/Empty` service (e.g., `/mast_open`, `/open_arm`) |
| `SleepRos` | StatefulAction | Non-blocking timed pause |
| `TimedMotion` | StatefulAction | Calls a start service, waits a duration, then calls a stop service |
| `ObstacleTooClose` | Condition | Returns SUCCESS when minimum LiDAR range < threshold |
| `PickRandomTurn` | SyncAction | Outputs `"left"` or `"right"` randomly |
| `RandomSuccess` | Condition | Returns SUCCESS with configurable probability |

An example tree (`silly_explorer.xml`) demonstrates a reactive exploration loop:
```
Root (Fallback)
├── IfFoundThenDone (Sequence) — 8% chance each tick: open/close mast ("photo")
└── ExploreOrAvoid (ReactiveFallback)
    ├── AvoidObstacle — if scan < 1m: stop → pick random turn → turn 1.2s
    └── ExploreCycle — pick turn → turn 0.8s → drive 5s → mast/arm survey
```

### `spacetry_battery` — Battery manager node

A standalone node that simulates rover battery state. It subscribes to `/cmd_vel`, `/joint_states`, and odometry to compute power consumption, and publishes:

| Topic | Type | Purpose |
|---|---|---|
| `/battery_state` | `sensor_msgs/BatteryState` | Full battery state (voltage, current, SOC, charging status) |
| `/battery/soc` | `std_msgs/Float32` | State-of-charge fraction (0.0–1.0) |
| `/battery/near_outpost` | `std_msgs/Bool` | `true` when rover is within `outpost_radius_m` of the outpost |

Key parameters: `capacity_wh` (120 Wh), `drain_base_w` (4 W idle), `drain_move_w_per_mps` (55 W per m/s), `outpost_recharge_w` (90 W near outpost). The BT can read `/battery/soc` to decide when to return for recharging.

### `spacetry_perception` — Obstacle direction classifier

Processes `/scan` (LaserScan) data and classifies obstacles into directional sectors using TF2 transforms into `base_link`:

| Topic | Type | Content |
|---|---|---|
| `obstacle/front` | `std_msgs/Bool` | Obstacle in front sector (±20°) |
| `obstacle/left` | `std_msgs/Bool` | Obstacle in left sector (60°–120°) |
| `obstacle/right` | `std_msgs/Bool` | Obstacle in right sector |
| `obstacle/state` | `std_msgs/String` | Priority label: `FRONT` > `LEFT` > `RIGHT` > `CLEAR` |

These topics are designed for BT condition nodes to make navigation decisions without raw scan processing.

### `spacetry_mission` — Mission configuration

YAML-based mission configuration consisting of three files:

- **`waypoints.yaml`** — Named navigation waypoints in world frame (used by bringup for spawn location and by BT for navigation goals):
  - `outpost_habitat_01` (66, 0), `solar_panel` (60, 1), `science_rock_01` (50.1, −80), `ridge` (12, 4), `safe_haven` (−5, −5)

- **`objects.yaml`** — World objects with semantic types (validated against the SDF by `validate_mission_config.py`):
  - `outpost_habitat_01` (comms_asset), `solar_panel` (outpost_asset), `science_rock_01` (science_target), `block_island` (hazard)

- **`mission_01.yaml`** — Ordered objective list:
  1. Navigate to `ridge`
  2. Inspect `science_rock_01`
  3. Navigate to `solar_panel`
  4. Return to `outpost_habitat_01`

### `spacetry_bringup` — Integration launch

The single launch file (`spacetry_curiosity_outpost.launch.py`) orchestrates the full mission stack:

1. Launches the `mars_outpost` world (GUI or headless via `headless` arg)
2. Publishes `robot_description` via `robot_state_publisher`
3. Spawns the Curiosity rover at the pose defined by the `spawn_waypoint` arg (default: `dock_pad_01` from `waypoints.yaml`, falls back to origin if missing)
4. Starts ROS↔Gazebo bridges (`/clock`, `/scan`, `/odometry`, `/image_raw`)
5. Loads the ros2_control controller chain (joint state broadcaster → arm/mast/wheel/steer/suspension controllers)
6. Launches demo nodes for arm/mast/wheel services
7. Starts `battery_manager` (initial SOC configurable via `battery` arg)
8. Starts `obstacle_direction_node` (LiDAR obstacle classification)


