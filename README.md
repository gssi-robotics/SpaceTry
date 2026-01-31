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