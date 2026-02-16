# Curiosity Mars World Extension

We have extended the [space ros curiosity demo repository world](https://github.com/space-ros/demos/blob/main/curiosity_rover/curiosity_gazebo/worlds/mars_curiosity.world). We added the following components from Gazebo:
- https://app.gazebosim.org/OpenRobotics/fuel/models/International%20Space%20Station
- https://app.gazebosim.org/OpenRobotics/fuel/models/Radio%20tower
- https://app.gazebosim.org/OpenRobotics/fuel/models/Falling%20Rock%203
- https://app.gazebosim.org/Cole/fuel/models/Ocean%20Rock%2001
- https://app.gazebosim.org/Cole/fuel/models/Ocean%20Rock%2007

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
