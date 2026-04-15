graph TB
    subgraph ROS2["ROS 2"]
        BT["Behavior Tree Runner<br/>spacetry_bt_runner"]
        Navigate["NavigateWithAvoidance<br/>Node"]
        Battery["Battery Manager<br/>spacetry_battery"]
        Perception["Perception Stack<br/>obstacle_direction"]
        JSB["Joint State Broadcaster"]
        MoveWheel["Move Wheel Adapter<br/>curiosity_rover_demo"]
        Controllers["ROS 2 Controllers<br/>wheel/steer/arm/mast"]
    end

    subgraph Gazebo["Gazebo Simulator"]
        World["Mars Outpost World<br/>Curiosity Rover"]
        Sensors["Sensors<br/>LiDAR, Camera, Odometry"]
        Physics["Physics Engine<br/>& Controllers"]
    end

    subgraph Bridge["ros_gz_bridge & ros_gz_image<br/>(Data Bridge Layer)"]
        ClockBridge["/clock"]
        OdomBridge["/model/curiosity_mars_rover/odometry"]
        ScanBridge["/scan"]
        ImageBridge["/image_raw"]
    end

    BT -->|defines goals| Navigate
    Navigate -->|publishes| CMD["/cmd_vel<br/>Twist"]
    CMD -->|consumed by| MoveWheel
    MoveWheel -->|converted to| Controllers
    Controllers -->|actuator commands<br/>wheel velocity, steering,<br/>arm/mast trajectories| Physics
    
    Sensors -->|odometry, lidar,<br/>camera data| OdomBridge
    Sensors -->|lidar ranges| ScanBridge
    Sensors -->|camera frames| ImageBridge
    Sensors -->|joint states| JSB
    
    OdomBridge -->|nav_msgs/Odometry| Navigate
    OdomBridge -->|odometry| Battery
    ScanBridge -->|sensor_msgs/LaserScan| Perception
    ScanBridge -->|lidar data| Navigate
    ImageBridge -->|sensor_msgs/Image| Perception
    JSB -->|sensor_msgs/JointState| Battery
    
    Gazebo -->|simulation time| ClockBridge
    ClockBridge -->|rosgraph_msgs/Clock| ROS2
    
    style BT fill:#4a90e2,color:#fff
    style Navigate fill:#4a90e2,color:#fff
    style Bridge fill:#f5a623,color:#000
    style Physics fill:#7ed321,color:#000
    style Sensors fill:#7ed321,color:#000

