World files (.sdf) will live here.

## Validate Gazebo World 
Recommended to run in case changes are introduced in the world to prevent unecessary GitHub Actions quota usage (in case of errors).
After building the docker container images, following the instructions [here](../../../README.md):

Inside the container, run:

### Setup

```bash
source /opt/ros/spaceros/setup.bash
source /etc/profile
colcon build --merge-install --event-handlers console_direct+
```

### Headless Validation (without ROS)
```bash 
GZ_SIM_RESOURCE_PATH=/ws/src/marti_models/models:/ws/src/space_ros_demos/curiosity_rover/curiosity_gazebo/models timeout 5 gz sim -s src/marti_world/worlds/mars_outpost.sdf
```

### GUI Validation (without ROS)
```bash 
GZ_SIM_RESOURCE_PATH=/ws/src/marti_models/models/:/ws/src/space_ros_demos/curiosity_rover/curiosity_gazebo/models gz sim src/marti_world/worlds/mars_outpost.sdf
```
