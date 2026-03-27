# Curiosity Mars World Extension

We have extended the [space ros curiosity demo repository world](https://github.com/space-ros/demos/blob/main/curiosity_rover/curiosity_gazebo/worlds/mars_curiosity.world) by adding custom models and functionalities needed to design our [mission description](../../spacetry_mission/MISSION.md).

## Gazebo models
Local models can be found in `[src/spacetry_models/models/](../../spacetry_models/)`:

| Model | Description |
|---|---|
| `station` | Outpost habitat module |
| `rock_5` | Lightweight science rock (static) |
| `block_island` | Hazard patch terrain obstacle (static) |

External models (imported at Docker build time from [space-ros/demos](https://github.com/space-ros/demos)):

| Model | Description |
|---|---|
| `curiosity_path` | Curiosity rover traversal path overlay |
| `curiosity_mars_rover` | Curiosity rover (spawned via [spacetry_bringup](../../spacetry_bringup/launch/spacetry_curiosity_outpost.launch.py)) |

## Gazebo worlds
| World | File |
|---|---|
| Mars Outpost | `src/spacetry_world/worlds/mars_outpost.sdf` |


## Validate Gazebo Assets 
Recommended to run in case changes are introduced in the world to prevent unecessary GitHub Actions quota usage (in case of errors).
To build and run the docker container images, follow the instructions [here](../../../README.md).

Inside the container, run:

### Setup

```bash
source /opt/ros/spaceros/setup.bash
source /etc/profile
colcon build --merge-install --event-handlers console_direct+
```

### Headless Validation of the Simulation World (without ROS)
```bash 
GZ_SIM_RESOURCE_PATH=/ws/src/spacetry_models/models:/ws/src/space_ros_demos/curiosity_rover/curiosity_gazebo/models timeout 10 gz sim -s src/spacetry_world/worlds/mars_outpost.sdf
```

### Headless Validation of the Simulation World + Rover
```bash 
./scripts/smoke_test.sh
```

### GUI Validation (without ROS)
```bash 
GZ_SIM_RESOURCE_PATH=/ws/src/spacetry_models/models:/ws/src/space_ros_demos/curiosity_rover/curiosity_gazebo/models gz sim src/spacetry_world/worlds/mars_outpost.sdf
```

### Validation with Curiosity Rover
```bash 
ros2 launch spacetry_bringup spacetry_curiosity_outpost.launch.py headless:=0
```