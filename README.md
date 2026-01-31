# MARTI — Mars Autonomous Rover Training Infrastructure

Course-grade Mars mission demo pack for Space ROS + Behavior Trees.

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
````
./scripts/run.sh
````

Inside the container:
```
source /opt/ros/spaceros/setup.bash
source /ws/install/setup.bash
ros2 launch marti_world mars_outpost.launch.py
```
> The launch file configures Gazebo resource paths so local models resolve without manually exporting environment variables.

## Mission configuration

Mission configuration files live in:
* ```src/marti_mission/config/waypoints.yaml``` — named navigation waypoints (frame: world)
* ```src/marti_mission/config/objects.yaml``` — mission objects (IDs must match model instance names in the world)
* ```src/marti_mission/config/mission_01.yaml``` — example objective list (optional)

### Validate mission config against the world
The validator checks that IDs in objects.yaml exist in the installed world SDF.
Run (from the repo root):
```
docker run --rm --platform linux/amd64 \
  -v "$(pwd)":/ws -w /ws \
  marti:dev bash -lc '
set -e
source /opt/ros/spaceros/setup.bash
colcon build
source install/setup.bash
export MARTI_WORLD_SDF="$(ros2 pkg prefix marti_world)/share/marti_world/worlds/mars_outpost.sdf"
export MARTI_MISSION_CONFIG_DIR="$(ros2 pkg prefix marti_mission)/share/marti_mission/config"
python3 /ws/scripts/validate_mission_config.py
'
```

## Project layout (reference)
* ```docker/``` — Dockerfile + compose + entrypoint
* ```scripts/``` — build/run/smoke test/validator
* ```src/``` — ROS 2 packages:
    * ```marti_world``` — Gazebo world(s)
    * ```marti_models``` — local Gazebo models
    * ```marti_mission``` — mission YAML configs