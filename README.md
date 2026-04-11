<h1>SpaceTry 🥐 — <u>Space</u> <u>T</u>raining <u>R</u>over Autonom<u>Y</u></h1>

SpaceTry is an infrastructure to train space rovers autonomy. This repository has a course-grade Mars mission demo pack for Space ROS + Behavior Trees + Autonomy Evaluation Scenario Generation Skill for LLM Agents. 

**Contents:**
- [Project Layout](#project-layout) — Repository structure and ROS2 packages
- [Dependencies & Integrations](#dependencies--integrations) — Integration with Space-ROS, Curiosity Rover, and BehaviorTree.CPP
- [Quickstart](#quickstart-docker-only) — Quick validation: build Docker image and run smoke tests
- [Step-by-Step: Run SpaceTry 🥐](#step-by-step-run-spacetry--with-the-curiosity-rover) — Complete walkthrough from container startup to rover control
- [Mission configuration](#spacetry--mission-configuration) — Mission objectives, waypoints, and validation
- [Test Scenarios](#test-scenarios-for-autonomy-evaluation) — Framework for autonomy evaluation and self-adaptation testing

## Project layout

```
spacetry/
├── AGENTS.md                        - LLM agents instructions and rules
├── README.md                        - project overview and usage
├── REF_SCENARIO.md                  - autonomy behavior description for the reference scenario
├── deps/                            - repository configuration for dependency management
├── docker/
├── scripts/                        - helper scripts to build, execution, and validation
├── skills/
│   └── spacetry-autonomy-scenario-driver/  - autonomy testing scenario LLM agent skill
├── src/                            - ROS 2 packages & Gazebo assets
│   ├── spacetry_battery/           - Battery manager node
│   ├── spacetry_bringup/           - Rover launch configurations
│   ├── spacetry_bt/                - Behavior tree runner node (C++)
│   ├── spacetry_mission/           - Mission description and configuration files
│   ├── spacetry_models/            - Gazebo models (target rocks, obstacles, outpost)
│   ├── spacetry_monitors/          - Safety properties monitoring node
│   ├── spacetry_perception/        - Perception nodes
│   └── spacetry_world/             - Gazebo world configurations
├── docs/                           - Project structure and implementation details
└── logs/                           - Scenario execution and test run logs
```

The full project implementation and structure details can be found in [IMPLEMENTATION.md](docs/IMPLEMENTATION.md).


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
<summary> 4. Launch SpaceTry Rover in Simulation with Autonomous Mission 🥐 </summary>

   SpaceTry 🥐 bringup launches the `mars_outpost` world, spawns the Curiosity rover (with proper simulation clock synchronization), starts ROS↔Gazebo bridges, loads the ros2_control controller chain, and **automatically** starts the behavior tree mission runner.

   **Option A: Run from inside the running container**

   From inside the container (see step 2.), simply launch the rover:
   ```bash
   source /opt/ros/spaceros/setup.bash && source /ws/install/setup.bash && ros2 launch spacetry_bringup spacetry_curiosity_outpost.launch.py
   ```

   The launch will:
   1. Start Gazebo with mars_outpost world
   2. Publish ROS↔Gazebo bridges (including `/clock` for simulation time)
   3. Spawn Curiosity rover (with 2-second delay to ensure clock is available)
   4. Load ros2_control controller chain
   5. Automatically start the behavior tree runner to execute the mission

   To use a different BT tree file:
   ```bash
   source /opt/ros/spaceros/setup.bash && source /ws/install/setup.bash && ros2 launch spacetry_bringup spacetry_curiosity_outpost.launch.py tree_file:=$(ros2 pkg prefix --share spacetry_bt)/trees/my_custom_tree.xml
   ```

   **Option B: Run from a new terminal window**

   ```bash
   docker exec -it docker-spacetry-1 bash -lc 'source /opt/ros/spaceros/setup.bash && source /ws/install/setup.bash && ros2 launch spacetry_bringup spacetry_curiosity_outpost.launch.py battery:=0.75'
   ```

   To specify a custom BT tree:
   ```bash
   docker exec -it docker-spacetry-1 bash -lc 'source /opt/ros/spaceros/setup.bash && source /ws/install/setup.bash && ros2 launch spacetry_bringup spacetry_curiosity_outpost.launch.py battery:=0.75 tree_file:=$(ros2 pkg prefix --share spacetry_bt)/trees/my_custom_tree.xml'
   ```

   **Option C: Run in Headless Mode (no Gazebo GUI)**

   ```bash
   docker exec -it docker-spacetry-1 bash -lc 'source /opt/ros/spaceros/setup.bash && source /etc/profile && source /ws/install/setup.bash && ros2 launch spacetry_bringup spacetry_curiosity_outpost.launch.py battery:=0.5 headless:=1'
   ```

   To specify a custom BT tree:
   ```bash
   docker exec -it docker-spacetry-1 bash -lc 'source /opt/ros/spaceros/setup.bash && source /etc/profile && source /ws/install/setup.bash && ros2 launch spacetry_bringup spacetry_curiosity_outpost.launch.py battery:=0.5 headless:=1 tree_file:=$(ros2 pkg prefix --share spacetry_bt)/trees/my_custom_tree.xml'
   ```

   **Option D: Launch without the Behavior Tree**

   To launch the rover without the BT runner (for manual testing or debugging):
   ```bash
   docker exec -it docker-spacetry-1 bash -lc 'source /opt/ros/spaceros/setup.bash && source /ws/install/setup.bash && ros2 launch spacetry_bringup spacetry_curiosity_outpost.launch.py enable_bt_runner:=false'
   ```

   This starts the full simulation stack without the autonomous mission runner. Useful for:
   - Manual rover control via command-line or external controllers
   - Testing individual components (perception, battery manager, etc.)
   - Debugging without BT execution overhead

   **Option E: Run the Behavior Tree separately on a new terminal**

   First, launch the rover without the BT runner:
   ```bash
   docker exec -it docker-spacetry-1 bash -lc 'source /opt/ros/spaceros/setup.bash && source /ws/install/setup.bash && ros2 launch spacetry_bringup spacetry_curiosity_outpost.launch.py enable_bt_runner:=false'
   ```

   Then in another terminal, run the behavior tree runner:
   ```bash
   docker exec -it docker-spacetry-1 bash -lc 'source /opt/ros/spaceros/setup.bash && source /ws/install/setup.bash && ros2 run spacetry_bt spacetry_bt_runner --ros-args -p tree_file:=$(ros2 pkg prefix --share spacetry_bt)/trees/base_bt.xml --params-file /ws/src/spacetry_bt/bt_params.yaml'
   ```

   This allows you to start the BT independently, giving you fine-grained control over when the mission execution begins.

   **Option F: Verify the rover and BT are running**

   Check if Curiosity nodes are active:
   ```bash
   docker exec -it docker-spacetry-1 bash -lc 'source /opt/ros/spaceros/setup.bash && ros2 node list | grep curiosity'
   ```

   Verify the behavior tree runner is executing:
   ```bash
   docker exec -it docker-spacetry-1 bash -lc 'source /opt/ros/spaceros/setup.bash && ros2 node list | grep spacetry_bt_runner'
   ```

   You should see:
   - Gazebo window with mars_outpost scene and rover spawned near `dock_pad_01`
   - Console output from rover nodes and BT runner showing mission execution (if BT enabled)
   - Use `battery:=0.5` to set initial battery state (example: 50%)
   - Use `enable_bt_runner:=false` to launch without the behavior tree
   - Use `tree_file:=$(ros2 pkg prefix --share spacetry_bt)/trees/my_tree.xml` to specify a custom BT tree
   - The BT executes the mission from [base_bt.xml](src/spacetry_bt/trees/base_bt.xml) automatically (when enabled)

</details>

<details>
<summary> 5. Stop SpaceTry 🥐 </summary>

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
   * ```src/spacetry_mission/config/objects.yaml``` — mission objects (IDs must match model instance names in the world)
   * ```src/spacetry_mission/config/waypoints.yaml``` — named navigation waypoints (frame: world)

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

## Test Scenarios for Autonomy Evaluation

SpaceTry 🥐 includes an LLM-Agent [Skill](skills) for generating and running test scenarios that evaluate the rover's autonomous capabilities and self-adaptation to faults or  changing/unforeseen conditions.

### Overview

Test scenarios inject uncertainty into the rover's mission execution to evaluate:
- **Perceptual Adaptation** — Can the rover adapt when sensors degrade or fail?
- **Behavioral Flexibility** — Does the behavior tree transition to contingencies appropriately?
- **Resource-Aware Autonomy** — Can the rover adapt goals to energy constraints?
- **Obstacle Intelligence** — Can the rover discover and avoid dynamic obstacles?
- **Mission Resilience** — Can the rover recover from failures or replanning challenges?
- **Safety Under Autonomy** — Does self-adaptation maintain safety constraints?


### Autonomy Test Scenario Workflow

For a deeper understanding of the test scenario process and how to create custom scenarios, see the [AGENTS.md](AGENTS.md) section on [Autonomy Test Scenario Generation and Evaluation Workflow](AGENTS.md#autonomy-test-scenario-generation-and-evaluation-workflow) and the [Scenario Generation Prompt Template](skills/spacetry-autonomy-scenario-driver/assets/SCENARIO_PROMPT_TEMPLATE.md).

The workflow consists of:

| Step | Task | Description | Workflow |
|------|------|-------------|----------|
| 1 | **Scenario Prompt Template** | Specify the autonomy under text and mission context and objectives using the prompt template. | User Input |
| 2 | **Scenario Driver Generation** | Use the LLM agent custom Skill to generate and parametrize the evaluation scenario from the prompt. | Automated |
| 3 | **Scenario Driver Parametrization** | Configure and fine-tune scenario parameters before execution. | Automated |
| 4 | **Scenario Driver Execution** | Execute the generated scenario in the simulation environment. | Automated |
| 5 | **Autonomy Evaluation Report** | Analyze the output report from the agent to assess the rover's self-adaptation capabilities in the uncertainty test scenario. | User Input |

A quick reference guide with examples is available at [SCENARIO_PROMPT_QUICK_REF.md](skills/spacetry-autonomy-scenario-driver/references/SCENARIO_PROMPT_QUICK_REF.md).
