# Space Robotics Use Case Description
## RoME Working Session 5 Mission Description
The idea is that you design a mission that explores an unknown environment surrounding the rover to find sampling locations when a payload is being triggered by a science instrument. In this case, the camera and the lidar. Figure 1 below depicts the overall idea of the mission, where the exploration goal is to find areas that align with the robot’s sampling and sensing capabilities.
![Intersection between the unknown and the known environments during robot's operation.](robotics_ops.png)
<p style="text-align: center;"> Fig. 1: Robotics mission goals.</p>

### Goals
The main goal of the deployed rovers is to conduct scientific exploration and sampling of Mars. Depending on the rover payload configuration, different types of samples  or sensor data can be collected. For the NASA Curiosity rover a detailed list of instruments is provided [here](https://science.nasa.gov/resource/diverse-science-payload-on-mars-rover-curiosity/). 


The goal of the rover mission will be to explore the surrounding environment starting from the landing point. To increase autonomy, the mission goal is to find the following resources to sample/sense:
1. Rock samples

An important aspect in the design to consider are the <u>terrain obstacles</u> and <u>battery energy levels</u>. Another requirements encompass guaranteeing that the sampling data is transmitted to the gateway in the outpost station. This can be done either by using a reliable transport layer protocol (e.g. TCP or HTTP), or by adding acknowledgment of the reception at the application layer.



## NASA Curiosity Rover

TBD

### Higher-Level Rover Tasks
1. Explore
    - Navigate safely towards unknown areas (not explored yet)
    - Circunvein big obstacles
2. Sampling
    - Identify (camera) and count samples
3. Recharge
    - Battery levels should be considered to determine the next action
4. Communicate
    - Asynchronous communication of findings (async Actions in ROS)

### Rover Payload
1. Arm with Camera
2. Lidar


