# Reference Scenario

Description of a reference scenario that implements a basic autonomy behavior via a behavior tree.

## Mission Goal
The goal of the rover is to navigate safely to a specified point of interest for sampling. Points of interest to be sampled during the exploration are configured as waypoints, and defined as goals in the behavior tree. The waypoint refers to a location of a rock to be sampled for scientific purposes. 

### Higher-Level Rover Tasks
1. **Explore** - NavigateWithAvoidance Maneuver (BT Action Node)
    - Navigate safely towards a goal waypoint (previously defined).
    
2. **Sample** - AlignToGoal + StopAndObserve Maneuver (BT Action Node)
    - Circumvent area around the goal waypoint

### Safety constraints
1. **Battery**
    - Battery levels should be considered to determine the next task and shall always be above 20% (topic /battery/soc 0.2). Otherwise, the rover shall return to the outpost.
2. **Wheel**
    - On good battery conditions (full), the rover speed should be above a certain threshold to avoid the wheel to get stuck.


### Behavior Tree
The example tree ([`base_bt.xml`](src/spacetry_bt/trees/base_bt.xml)) demonstrates the primary mission workflow:
```
MissionSequence (Sequence)
├── SetGoal — Set target waypoint to "science_rock"
├── NavigateWithAvoidance — Navigate to goal with LiDAR obstacle avoidance
│                           (v_lin=1.6 m/s, v_ang=1.4 rad/s, dist_tol=1.5 m, reverse_speed=0.8 m/s)
├── AlignToGoal — Align rover to final goal orientation
│                 (v_ang=1.0 rad/s, yaw_tol=10°)
├── StopAndObserve — Pause for 3 seconds at the goal location
└── LogMessage — Log "Science rock reached"
```