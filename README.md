# MARTI — Mars Autonomous Rover Training Infrastructure

Course-grade Mars mission demo pack for Space ROS + Behavior Trees.

## Project layout (reference)
* ```docker/``` — Dockerfile + compose + entrypoint
* ```scripts/``` — build/run/smoke test/validator
* ```src/``` — ROS 2 packages:
    * ```marti_world``` — Gazebo world(s)
    * ```marti_models``` — local Gazebo models
    * ```marti_mission``` — mission YAML configs
    
## Quick start (coming soon)
- Build: `./scripts/build.sh`
- Run:   `./scripts/run.sh`

## Quickstart (Docker-only)

MARTI is designed to run without installing ROS locally. Everything runs inside the MARTI Docker image.

### 1) Build the image
From the repo root:

```bash
./scripts/build.sh
```

### 2) Verify your setup

Run the smoke test (builds the workspace if needed, loads the world headless, validates configs):
```
./scripts/smoke_test.sh
```

You should see:
 * ```OK: mars_outpost loaded headless```
 * ```OK: mission config validated```

### Run the environment

Start a container shell using the provided script:
````bash
./scripts/run.sh
````

Enter the container: 
```bash
docker exec -it docker-marti-1 bash 
```

Insider the container, build:
```bash
source /opt/ros/spaceros/setup.bash
source /etc/profile 
colcon build --merge-install --event-handlers console_direct+
```

Insider the container, run:
```bash
source /ws/install/setup.bash
ros2 launch marti_world mars_outpost.launch.py
```
> The launch file configures Gazebo resource paths so local models resolve without manually exporting environment variables.

## Run MARTI with the Curiosity Mars Rover (GUI or headless)

MARTI includes a bringup that launches:
- the `mars_outpost` world
- the Curiosity rover (spawned near `dock_pad_01`)
- ROS↔Gazebo bridges + demo nodes

### 1) Start the container
From the repo root:
```bash
./scripts/run.sh
```

### 2) Build the workspace (first time, or after pulls/changes)
Inside the container:
```bash
cd /ws
colcon build --merge-install --event-handlers console_direct+
```

### 3) Launch Curiosity in the outpost world

#### Option A — With Gazebo GUI (recommended for manual driving)
Inside the container:
```bash
source /etc/profile
ros2 launch marti_bringup marti_curiosity_outpost.launch.py headless:=0
```

You should see Gazebo open with the outpost scene and the rover spawned nearby.

#### Option B — Headless (CI / remote / no rendering)
Inside the container:
```bash
source /etc/profile
ros2 launch marti_bringup marti_curiosity_outpost.launch.py headless:=1
```

### 4) Drive the rover using the Gazebo GUI

Once Gazebo is running with the rover spawned:

1. In Gazebo, open the **Entity Tree** and select the rover model:
   - Look for: `curiosity_mars_rover`
2. Open **Component Inspector** (or the right-side inspector) for the rover.
3. Find the rover’s velocity/command controls (typically under a plugin/controller section).
   - If you see fields for linear/angular velocity (or wheel/joint commands), adjust them live and apply.
4. Verify movement visually (GUI) or via ROS topics (headless/GUI):
   ```bash
   ros2 topic echo /model/curiosity_mars_rover/odometry --once
   ```

> Tip: If you don’t see any control widgets in the GUI inspector, make sure controllers loaded successfully in the launch logs (you should see the controller_manager calls complete). If they didn’t, re-run after `colcon build --merge-install` and ensure `source /etc/profile` before launching.

### 5) Useful runtime checks

List Curiosity-related ROS nodes:
```bash
ros2 node list | grep -E "curiosity|ros_gz|robot_state|controller" || true
```

Confirm bridges are up:
```bash
ros2 topic list | grep -E "^/clock$|^/scan$|/odometry" || true
```

Confirm sim time:
```bash
ros2 param get /robot_state_publisher use_sim_time
```

## Mission configuration

Mission configuration files live in:
* ```src/marti_mission/config/waypoints.yaml``` — named navigation waypoints (frame: world)
* ```src/marti_mission/config/objects.yaml``` — mission objects (IDs must match model instance names in the world)
* ```src/marti_mission/config/mission_01.yaml``` — example objective list (optional)

### Validate mission config against the world
The validator checks that IDs in objects.yaml exist in the installed world SDF.
Run (from the repo root):
```
docker run --rm --platform linux/amd64   -v "$(pwd)":/ws -w /ws   marti:dev bash -lc '
set -e
source /opt/ros/spaceros/setup.bash
colcon build
source install/setup.bash
export MARTI_WORLD_SDF="$(ros2 pkg prefix marti_world)/share/marti_world/worlds/mars_outpost.sdf"
export MARTI_MISSION_CONFIG_DIR="$(ros2 pkg prefix marti_mission)/share/marti_mission/config"
python3 /ws/scripts/validate_mission_config.py
'
```