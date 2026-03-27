<h1>SpaceTry 🥐 — <u>Space</u> <u>T</u>raining <u>R</u>over Autonom<u>Y</u></h1>

SpaceTry is an infrastructure to train space rovers autonomy. This repository has a course-grade Mars mission demo pack for Space ROS + Behavior Trees. 

Contents:
- [Project Layout](#project-layout)
- [Dependencies & Requirements](#dependencies--requirements)
- [Quickstart](#quickstart-docker-only)
- [Step-by-Step: Run SpaceTry 🥐](#step-by-step-run-spacetry--with-the-curiosity-rover)
- [Mission configuration](#spacetry--mission-configuration)

## Project layout
* `docker/` — Dockerfile + compose + entrypoint
* `scripts/` — build / run / smoke test / validator
* `deps/` — pinned external repos (`spacetry.repos`)
* `src/` — ROS 2 packages & Gazebo assets:

   * `spacetry_world` — Extension of the space-ros Curiosity Mars rover world and launch files ([description](src/spacetry_world/README.md))
   * `spacetry_models` — Gazebo models ( [description](src/spacetry_models/README.md))
   * `spacetry_bringup` — integration launch files (rover + world + bridges)
   * `spacetry_bt` — BehaviorTree.CPP-based mission runner
   * `spacetry_mission` — mission YAML configs and tooling
   * `spacetry_battery` — battery manager node
   * `spacetry_perception` — perception helpers (LiDAR obstacle direction)

## Dependencies & Integrations

SpaceTry 🥐 is designed to run without installing ROS locally. Everything runs inside the SpaceTry 🥐 Docker image on linux.

It integrates with i) The Curiosity Rover from [Space-ROS Demos](https://github.com/space-ros/demos/tree/main) and ii) BehaviorTree.CPP which trees can be used in [Groot2](https://www.behaviortree.dev/groot/).

<details>
<summary> NASA Curiosity Rover and World from Space-ROS </summary>

The Curiosity rover model comes from the [Space ROS Demos](https://github.com/space-ros/demos/tree/main/curiosity_rover) repository and is imported via `vcs` using `deps/spacetry.repos`. The import brings three ROS 2 packages:

| Package | Role |
|---|---|
| `curiosity_description` | URDF/xacro model of the rover (meshes, joints, sensors, ros2_control config) |
| `curiosity_gazebo` | Gazebo models (terrain, paths), plugins, and the `odom_tf_publisher` helper node |
| `curiosity_rover_demo` | Demo nodes for arm, mast, wheel control and the `mars_rover.launch.py` integration launch |

At launch, `spacetry_bringup` processes the xacro (`curiosity_mars_rover.xacro`) into a URDF XML and spawns the rover into the `mars_outpost` world via `ros_gz_sim create`. After spawn, the launch file loads a controller chain through `ros2_control`:

1. `GazeboSystem` hardware interface → active
2. `joint_state_broadcaster` → active
3. In parallel: `arm_joint_trajectory_controller`, `mast_joint_trajectory_controller`, `wheel_velocity_controller`, `steer_position_controller`, `wheel_tree_position_controller`

</details>

<details>
<summary> BehaviorTree.CPP and Groot2 Integration </summary>

SpaceTry 🥐 uses [BehaviorTree.CPP](https://www.behaviortree.dev/) (v4.6+) for mission logic and supports visual editing and monitoring via [Groot2](https://github.com/BehaviorTree/Groot2).

**Core Integration:**

The `spacetry_bt_runner` executable loads a BehaviorTree from an XML file and executes it with configurable tick rate (default 10 Hz). The runner:
- Loads XML tree from `tree_file` parameter
- Registers custom ROS 2 action and condition nodes for rover control
- Ticks the tree at configurable rate (via `tick_hz` parameter)
- Publishes ongoing execution status to stdout logs
- Supports max runtime limit via `max_runtime_s` parameter
- Reads waypoints and mission parameters from ROS 2 parameter server

**Custom BT Nodes:**

The runner provides mission-specific nodes registered with the BehaviorTree factory:

| Node | Type | Inputs | Purpose |
|---|---|---|---|
| `SetGoal` | SyncAction | `waypoint` (string) | Load a waypoint into the blackboard as current goal |
| `NavigateWithAvoidance` | StatefulAction | `goal`, `v_lin`, `min_lin`, `v_ang`, `dist_tol`, `obstacle_threshold_m` | Navigate towards goal while avoiding LiDAR obstacles |
| `AlignToGoal` | StatefulAction | `goal`, `v_lin`, `min_lin`, `v_ang`, `yaw_tol_deg` | Align rover orientation to goal heading |
| `StopAndObserve` | StatefulAction | `seconds` | Stop movement and pause (for sampling/observation) |
| `LogMessage` | SyncAction | `message` (string) | Log a status message to console |

**Groot2 Workflow:**

Trees are defined in standard BehaviorTree.CPP XML format (`BTCPP_format="4"`):
- Edit/visualize trees in **Groot2** (available at https://www.behaviortree.dev/groot/)
- Export to XML and place in `/src/spacetry_bt/trees/`
- Run via `spacetry_bt_runner` with `--ros-args -p tree_file:=<path-to-xml>`
- Monitor execution in real-time via stdout logs and ROS 2 topics

**Example Groot2 Export:**
Groot2 can monitor and edit trees during runtime if the BT is launched with the JSON logger enabled. For offline editing, export the tree as XML, which is automatically discovered by `colcon`'s install step.

</details>

<details>
<summary> Integration Launch</summary>

The single launch file (`spacetry_curiosity_outpost.launch.py`) in the package `spacetry_bringup` orchestrates the full mission stack:

1. Launches the `mars_outpost` world (GUI or headless via `headless` arg)
2. Publishes `robot_description` via `robot_state_publisher`
3. Spawns the Curiosity rover at the pose defined by the `spawn_waypoint` arg (default: `dock_pad_01` from `waypoints.yaml`, falls back to origin if missing)
4. Starts ROS↔Gazebo bridges (`/clock`, `/scan`, `/odometry`, `/image_raw`)
5. Loads the ros2_control controller chain (joint state broadcaster → arm/mast/wheel/steer/suspension controllers)
6. Launches demo nodes for arm/mast/wheel services
7. Starts `battery_manager` (initial SOC configurable via `battery` arg)
8. Starts `obstacle_direction_node` (LiDAR obstacle classification)
</details>


Please follow the instruction [here](https://docs.docker.com/engine/install/) to install Docker.

## Quickstart (Docker-only)

Build and run the smoke test (builds the docker image and the workspace, loads the world headless, validates configs).

### Build the image and verify your setup
From the repo root:

```bash
./scripts/build.sh && docker run --rm --platform linux/amd64 spacetry:dev /ws/scripts/smoke_test.sh
```

You should see:
 * ```OK: mars_outpost loaded headless```
 * ```OK: mission config validated```

## Step-by-Step: Run SpaceTry 🥐 with the Curiosity Rover

<details>
<summary> 1. Build the SpaceTry 🥐 Docker image </summary>

   From the repo root:

   ```bash
   ./scripts/build.sh
   ```

</details>


<details>
<summary> 2. Start the container and enter bash </summary>

   From the repo root:

   ```bash
   ./scripts/run.sh
   ```

   Enter the running container:

   ```bash
   docker exec -it docker-spacetry-1 bash 
   ```

</details>

<details>
<summary> 3. Build the workspace </summary>

   Inside the container, run:

   ```bash
   source /opt/ros/spaceros/setup.bash && source /etc/profile && colcon build --merge-install --event-handlers console_direct+
   ```

</details>

<details>
<summary> 4. Launch SpaceTry 🥐 with the Curiosity Mars Rover </summary>

   SpaceTry 🥐 includes a bringup that launches the `mars_outpost` world, the Curiosity rover (spawned near `dock_pad_01`), and ROS↔Gazebo bridges.

   In the same terminal:

   ```bash
   source /opt/ros/spaceros/setup.bash && source /ws/install/setup.bash && ros2 launch spacetry_bringup spacetry_curiosity_outpost.launch.py
   ```

   **Optional** -- If you want to launch from a new terminal:

   ```bash
   docker exec -it docker-spacetry-1 bash -lc 'source /opt/ros/spaceros/setup.bash && source /ws/install/setup.bash && ros2 launch spacetry_bringup spacetry_curiosity_outpost.launch.py'
   ```

   You should see Gazebo open with the outpost scene and rover spawned nearby. Use launch argument `battery:=0.5` to set initial battery state-of-charge (example: 50%).

   **Optional - Headless mode (no Gazebo GUI):**
   ```bash
   docker exec -it docker-spacetry-1 bash -lc 'source /opt/ros/spaceros/setup.bash && source /etc/profile && source /ws/install/setup.bash && ros2 launch spacetry_bringup spacetry_curiosity_outpost.launch.py battery:=0.5 headless:=1'
   ```

   **Verify rover is running:**
   ```bash
   docker exec -it docker-spacetry-1 bash -lc 'source /opt/ros/spaceros/setup.bash && ros2 node list | grep curiosity'
   ```

</details>

<details>
<summary> 5. Run the Behavior Tree (BT) or drive manually </summary>

   **Option A: Drive the rover manually in Gazebo GUI**
   - Open the Entity Tree and select `curiosity_mars_rover`
   - Open Component Inspector and adjust velocity commands

   **Option B: Run the Behavior Tree autonomously**

   In another terminal:

   ```bash
   docker exec -it docker-spacetry-1 bash -lc 'source /opt/ros/spaceros/setup.bash && source /ws/install/setup.bash && ros2 run spacetry_bt spacetry_bt_runner --ros-args -p tree_file:=$(ros2 pkg prefix --share spacetry_bt)/trees/base_bt.xml --params-file /ws/src/spacetry_bt/bt_params.yaml'
   ```

   The BT runner will execute the mission defined in `/ws/src/spacetry_bt/trees/base_bt.xml`.

</details>

<details>
<summary> 6. Stop SpaceTry 🥐 </summary>

After closing Gazebo GUI, exit all the containers bash with:

```bash
exit
```

And then from the repo root:

```bash
docker compose -f docker/docker-compose.yaml down
```

</details>

## SpaceTry 🥐 Mission Configuration

   Mission configuration files live in:
   * ```src/spacetry_mission/config/waypoints.yaml``` — named navigation waypoints (frame: world)
   * ```src/spacetry_mission/config/objects.yaml``` — mission objects (IDs must match model instance names in the world)
   * ```src/spacetry_mission/config/mission_01.yaml``` — example objective list (optional)

   ### Validate mission config against the world
   The validator checks that IDs in objects.yaml exist in the installed world SDF.
   Run (from the repo root):
   ```
   docker run --rm --platform linux/amd64   -v "$(pwd)":/ws -w /ws   spacetry:dev bash -lc '
   set -e
   source /opt/ros/spaceros/setup.bash
   colcon build
   source install/setup.bash
   export SPACETRY_WORLD_SDF="$(ros2 pkg prefix spacetry_world)/share/spacetry_world/worlds/mars_outpost.sdf"
   export SPACETRY_MISSION_CONFIG_DIR="$(ros2 pkg prefix spacetry_mission)/share/spacetry_mission/config"
   python3 /ws/scripts/validate_mission_config.py
   '
   ```
> More details on the mission goals, configurations, and launch can be found in [MISSION.md](src/spacetry_mission/MISSION.md).