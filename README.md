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

## Dependencies & Requirements

SpaceTry 🥐 is designed to run without installing ROS locally. Everything runs inside the SpaceTry 🥐 Docker image on linux.

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