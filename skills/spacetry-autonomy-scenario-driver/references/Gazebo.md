# Gazebo 

This document specifies Gazebo simulation tool features relevant for the SpaceTry project. Specific guidelines are provided to be considered when generating Gazebo-related software components for the autonomy test scenario driver.

## Capabilities 

Gazebo in this project uses SDF worlds to describe the simulation environment and its runtime behavior. The world file is the place where global physics settings, models, lights, and simulation plugins are declared, which makes it the main entry point for configuring terrain conditions, object layout, and world-level services used by the rover scenarios. The project uses the Harmonic version.

### World Configurations

To configure the robot simulation environment the following Gazebo features can be used:

- World Physics:
    - Physics engine and solver settings: the SDF `<physics>` block defines the physics engine configuration and numerical stepping behavior used to simulate rover motion and contact response.
    - Time-step and real-time control: world physics parameters define `max_step_size`, `real_time_factor`, and related execution settings, which directly affect simulation speed, stability, and timing-sensitive autonomy behavior.
    - Gravity: the world-level gravity vector is declared in SDF and, in SpaceTry, is configured to match Mars surface gravity.
    - Contact and collision response: physics and collision properties in world and model elements determine how terrain and objects interact with the rover, including contact generation, friction, and navigation-relevant surface behavior.
    - Lighting and scene conditions: world light sources are declared in SDF and influence rendering, visibility, and camera-like sensor perception in the simulation.

- Object Placement:
    - Type of configuration:
        - Static: loaded via SDF world description.
        - Dynamic: spawned in runtime but needs model needs to be loaded in the simulation environment.
    - Parameters:
        - Geometry: 
            - Primitive Gazebo Shapes: boxes, spheres, cylinders
            - Custom mesh and URDF models: available in the spacetry_models and the imported  curiosity_description packages
        - Pose:
            - Position: (x,y,z) coordinates with respect with the world frame.
            - Orientation: (roll, pitch, yaw) orientation with respect with the world frame.
        - Physics Properties:
            - Mass: object mass
            - Friction: useful for control and navigation
            - Contact: objects can be collided with or without contact

### Sensors

Gazebo Harmonic sensors are defined in SDF under model links or frames and publish simulation data through configured topics. For this project, the most relevant capability is that sensor behavior can be controlled through SDF parameters such as topic name, update rate, visualization, pose, and sensor-specific measurement ranges, which makes sensors a practical target for uncertainty injection and observability during autonomy evaluation.

- Sensor declaration:
    - Sensors are attached to robot links or frames inside SDF models.
    - Sensor system plugins must be enabled in the world when required by the sensor type.
- Common properties:
    - Topic: defines where simulated measurements are published.
    - Update rate: controls how frequently measurements are produced.
    - Pose: defines the sensor position and orientation relative to the robot structure.
    - Visualization: enables rendering the sensor output in Gazebo for debugging.
- Relevant sensor families for scenario design:
    - IMU: orientation, angular velocity, and linear acceleration signals.
    - Contact: collision-triggered signals for obstacle or interaction events.
    - Lidar: range-based obstacle perception with configurable scan angles, samples, and min/max range.

## Implementation Guidelines
- World features should be preferably implemented at design time, i.e., during the generation of autonomy test scenario driver, changing the properties in runtime to trigger adaptation.
- Best performant implementation strategy should be considered when implementing Gazebo-related uncertainty.

## Non-Negotiables
- The Gazebo world configured gravity and lighting are matching the conditions in Mars and should not be changed at any point of the implementation.
- The existing sensor suite for the rover in the project should not be changed at any point of the development or runtime testing.
- Sensor-related properties such as topics, pose, or scan angles should not be changed at any point of the development or runtime testing.
- Any other sensor-related properties being using in the autonomy specification source code, which is under test, should not be changed at any point of the development or runtime testing.

## Code Style and Guidelines
- For uncertainty injection in the simulation environment, existing models in the spacetry_models are prefered since they are more realistic. 
- For additional details about Gazebo Harmonic features and good practices can be found in the official documentation in: https://gazebosim.org/docs/harmonic/getstarted/. 
- Follow additional instructions in `AGENTS.md` files from project-wide and package-specific (in the `src/` sub-folders).
- In case of doubt, conflicting or missing information, ask clarification from the user.
