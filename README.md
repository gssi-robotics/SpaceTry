<h1>SpaceTry 🥐 — <u>Space</u> <u>T</u>raining <u>R</u>over Autonom<u>Y</u></h1>

SpaceTry is an infrastructure to train space rovers autonomy. This repository has a course-grade Mars mission demo pack for Space ROS + Behavior Trees + Autonomy Evaluation Scenario Genaration Skill for LLM Agents. 

**Contents:**
- [Project Layout](#project-layout) — Repository structure and ROS2 packages
- [Dependencies & Integrations](#dependencies--integrations) — Integration with Space-ROS, Curiosity Rover, and BehaviorTree.CPP
- [Quickstart](#quickstart-docker-only) — Quick validation: build Docker image and run smoke tests
- [Step-by-Step: Run SpaceTry 🥐](#step-by-step-run-spacetry--with-the-curiosity-rover) — Complete walkthrough from container startup to rover control
- [Mission configuration](#spacetry--mission-configuration) — Mission objectives, waypoints, and validation
- [Test Scenarios](#test-scenarios-for-autonomy-evaluation) — Framework for autonomy evaluation and self-adaptation testing

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
   * ```src/spacetry_mission/config/mission_01.yaml``` — mission objectives
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

SpaceTry 🥐 includes a framework for generating and running test scenarios that evaluate the rover's autonomous capabilities and self-adaptation to changing/unforeseen conditions.

### Overview

Test scenarios inject uncertainty into the rover's mission execution to evaluate:
- **Perceptual Adaptation** — Can the rover adapt when sensors degrade or fail?
- **Behavioral Flexibility** — Does the behavior tree transition to contingencies appropriately?
- **Resource-Aware Autonomy** — Can the rover adapt goals to energy constraints?
- **Obstacle Intelligence** — Can the rover discover and avoid dynamic obstacles?
- **Mission Resilience** — Can the rover recover from failures or replanning challenges?
- **Safety Under Autonomy** — Does self-adaptation maintain safety constraints?

### Quick Start: Run a Test Scenario

<details>
<summary> 1. Understand the Test Scenario Framework </summary>

   Scenario templates are available in the `scenarios/` directory:

   - [**SCENARIO_PROMPT_TEMPLATE.md**](scenarios/SCENARIO_PROMPT_TEMPLATE.md) — Comprehensive guide with base template, instantiation examples, and usage patterns
   - [**SCENARIO_PROMPT_QUICK_REF.md**](scenarios/SCENARIO_PROMPT_QUICK_REF.md) — Quick reference with one-liners, uncertainty injection templates, and naming conventions

   These templates guide the creation of scenario driver components that:
   - Inject uncertainty at decision-critical moments
   - Monitor rover autonomy metrics (goal completion, contingency activations, safety violations)
   - Log behavior for post-execution analysis
   - Adapt challenge intensity based on observed performance

</details>

<details>
<summary> 2. Create a Test Scenario Description </summary>

   Use the scenario prompt templates to define your test scenario. For example:

   **Sensor Degradation Scenario:**
   ```
   Given the behavior tree at src/spacetry_bt/trees/base_bt.xml,
   source code in src/spacetry_perception/,
   and mission defined in src/spacetry_mission/config/mission_01.yaml,
   create a scenario driver that injects LIDAR degradation (100% → 50% → fail over 5 min)
   to test whether autonomous perception adaptation can maintain obstacle avoidance.
   ```

   **Dynamic Obstacle Scenario:**
   ```
   Given the behavior tree at src/spacetry_bt/trees/base_bt.xml,
   source code in src/spacetry_perception/,
   and mission defined in src/spacetry_mission/config/mission_01.yaml,
   create a scenario driver that spawns dynamic obstacles at waypoints
   to test whether autonomous navigation can detect, replan, and avoid.
   ```

   **Power Constraint Scenario:**
   ```
   Given the behavior tree at src/spacetry_bt/trees/base_bt.xml,
   source code in src/spacetry_battery/battery_manager_node.py,
   and mission defined in src/spacetry_mission/config/mission_01.yaml,
   create a scenario driver that accelerates battery drain
   to test whether autonomous mission planning can gracefully degrade and complete with limited energy.
   ```

   See [scenarios/SCENARIO_PROMPT_QUICK_REF.md](scenarios/SCENARIO_PROMPT_QUICK_REF.md) for rapid scenario generation templates.

</details>

<details>
<summary> 3. Build and Prepare the Scenario </summary>

   From inside the running container:

   ```bash
   # Rebuild packages if scenario components have changed
   docker compose exec spacetry colcon build --packages-select spacetry_mission spacetry_bt spacetry_world spacetry_perception spacetry_battery
   ```

</details>

<details>
<summary> 4. Launch the Rover with Scenario Parameters </summary>

   Scenarios are launched using the same bringup launch file with additional parameters to enable uncertainty injection:

   ```bash
   docker compose exec spacetry bash -lc '\
     source /opt/ros/spaceros/setup.bash && source /ws/install/setup.bash && \
     ros2 launch spacetry_bringup spacetry_curiosity_outpost.launch.py \
       headless:=0 \
       battery:=1.0 \
       spawn_waypoint:=dock_pad_01 \
       scenario_enabled:=true \
       scenario_type:=sensor_degradation
   '
   ```

   **Available scenario parameters:**
   - `scenario_enabled` (true/false) — Enable uncertainty injection
   - `scenario_type` — Type of uncertainty: `sensor_degradation`, `dynamic_obstacles`, `power_constraints`, `cascading_failures`
   - `scenario_intensity` (float 0-1) — Intensity of uncertainty injection (0=none, 1=maximum)
   - `scenario_log_dir` (string) — Output directory for scenario logs (default: `/tmp/spacetry_scenarios/`)

   **Example scenarios:**

   *Scenario 1: Progressive LIDAR Degradation*
   ```bash
   docker compose exec spacetry bash -lc '\
     source /opt/ros/spaceros/setup.bash && source /ws/install/setup.bash && \
     ros2 launch spacetry_bringup spacetry_curiosity_outpost.launch.py \
       headless:=0 battery:=1.0 \
       scenario_enabled:=true \
       scenario_type:=sensor_degradation \
       scenario_intensity:=0.5
   '
   ```

   *Scenario 2: Dynamic Obstacles*
   ```bash
   docker compose exec spacetry bash -lc '\
     source /opt/ros/spaceros/setup.bash && source /ws/install/setup.bash && \
     ros2 launch spacetry_bringup spacetry_curiosity_outpost.launch.py \
       headless:=0 battery:=1.0 \
       scenario_enabled:=true \
       scenario_type:=dynamic_obstacles \
       scenario_intensity:=0.7
   '
   ```

   *Scenario 3: Power Constraints*
   ```bash
   docker compose exec spacetry bash -lc '\
     source /opt/ros/spaceros/setup.bash && source /ws/install/setup.bash && \
     ros2 launch spacetry_bringup spacetry_curiosity_outpost.launch.py \
       headless:=0 battery:=0.5 \
       scenario_enabled:=true \
       scenario_type:=power_constraints \
       scenario_intensity:=0.8
   '
   ```

</details>

<details>
<summary> 5. Run the Behavior Tree During Scenario Execution </summary>

   In another container terminal, start the behavior tree runner:

   ```bash
   docker compose exec spacetry bash -lc '\
     source /opt/ros/spaceros/setup.bash && source /ws/install/setup.bash && \
     ros2 run spacetry_bt spacetry_bt_runner \
       --ros-args \
       -p tree_file:=$(ros2 pkg prefix --share spacetry_bt)/trees/base_bt.xml \
       -p tick_hz:=10 \
       -p max_runtime_s:=300 \
       --params-file /ws/src/spacetry_bt/bt_params.yaml
   '
   ```

   The behavior tree will execute the mission objectives from `src/spacetry_mission/config/mission_01.yaml` while the scenario driver injects uncertainty at decision points.

</details>

<details>
<summary> 6. Monitor Autonomy Metrics During Execution </summary>

   While the scenario runs, monitor key autonomy metrics in a third terminal:

   **Goal Progress:**
   ```bash
   docker compose exec spacetry bash -lc '\
     source /opt/ros/spaceros/setup.bash && source /ws/install/setup.bash && \
     ros2 topic echo /spacetry_mission/current_objective
   '
   ```

   **Battery State:**
   ```bash
   docker compose exec spacetry bash -lc '\
     source /opt/ros/spaceros/setup.bash && source /ws/install/setup.bash && \
     ros2 topic echo /battery_manager/state_of_charge
   '
   ```

   **Obstacle Detection (if applicable):**
   ```bash
   docker compose exec spacetry bash -lc '\
     source /opt/ros/spaceros/setup.bash && source /ws/install/setup.bash && \
     ros2 topic echo /obstacle_direction
   '
   ```

   **Behavior Tree Status:**
   ```bash
   docker compose exec spacetry bash -lc '\
     source /opt/ros/spaceros/setup.bash && source /ws/install/setup.bash && \
     ros2 topic echo /tree_outputs
   '
   ```

</details>

<details>
<summary> 7. Analyze Scenario Results </summary>

   After the scenario completes, analyze the results using the scenario report:

   ```bash
   docker compose exec spacetry bash -lc '\
     source /opt/ros/spaceros/setup.bash && source /ws/install/setup.bash && \
     python3 /ws/scripts/scenario_analyzer.py \
       --log-dir /tmp/spacetry_scenarios/latest \
       --output-report /tmp/spacetry_scenarios/latest/autonomy_report.md
   '
   ```

   The report will show:
   - **Autonomy Metrics**: Adaptation success rate, contingency activations, recovery time
   - **Goal Completion**: Primary objectives reached, partial completion, failure modes
   - **Safety Constraints**: Violations detected, boundary breaches, collision events
   - **Graceful Degradation**: Performance under increasing uncertainty intensity
   - **Recommendations**: Which autonomy aspects to improve based on scenario results

</details>

### Test Scenario Workflow (Detailed)

For a deeper understanding of the test scenario process and how to create custom scenarios, see the [AGENTS.md](AGENTS.md) section on [Test Scenario Generation Workflow](AGENTS.md#test-scenario-generation-workflow).

The workflow consists of:
1. **Define Autonomy Test Context** — Specify scenario parameters and objectives
2. **Build and Prepare** — Compile scenario components
3. **Launch Scenario with Autonomy Variations** — Run rover with uncertainty injection
4. **Execute Test & Monitor Autonomous Behavior** — Observe real-time decision-making
5. **Analyze Autonomy Results** — Generate evaluation report with metrics

### Scenario Template Files

Quick references for scenario creation:
- [scenarios/SCENARIO_PROMPT_TEMPLATE.md](scenarios/SCENARIO_PROMPT_TEMPLATE.md) — Comprehensive template with examples
- [scenarios/SCENARIO_PROMPT_QUICK_REF.md](scenarios/SCENARIO_PROMPT_QUICK_REF.md) — Quick reference guide
- [AGENTS.md](AGENTS.md) — Project-wide autonomy evaluation framework
- [src/spacetry_mission/config/mission_01.yaml](src/spacetry_mission/config/mission_01.yaml) — Example mission configuration
- [src/spacetry_mission/config/waypoints.yaml](src/spacetry_mission/config/waypoints.yaml) — Navigation waypoints
- [src/spacetry_mission/config/objects.yaml](src/spacetry_mission/config/objects.yaml) — Mission objects and hazards