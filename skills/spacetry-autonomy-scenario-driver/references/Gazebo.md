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

### Baseline World Awareness

Scenario generation must treat the active Gazebo world as a populated mission map, not a blank canvas.

- Inspect the active world SDF before planning uncertainty injection or mission deadlines.
- Identify mission-relevant entities that already exist in the world, including baseline goals, landmarks, hazards, and route blockers.
- Cross-check those entities against the mission waypoint configuration and the user prompt or reference scenario so the generated scenario evaluates the intended baseline goal rather than a duplicate or conflicting one.
- If the prompt names an existing target, such as a science rock already placed in the world, reuse that target as the evaluated goal unless the user explicitly asks for a different one.
- When placing new runtime obstacles, justify their placement relative to the existing world layout and nominal route instead of assuming open terrain.
- When defining timeouts or deadlines, account for the actual start pose, waypoint distance, and known baseline obstacles already present in the world.

### Runtime Model Insertion

Gazebo runtime model insertion is a valid mechanism for uncertainty injection in autonomy scenarios, but it requires explicit validation.

- Dynamic model insertion:
    - `ros_gz_sim create` can spawn models from an installed file path during scenario execution.
    - A successful process exit code is not sufficient evidence that the model is usable in the world.
- Resource path handling:
    - When a scenario driver spawns a model from a file at runtime, the subprocess should explicitly provide `GZ_SIM_RESOURCE_PATH` and `SDF_PATH` so Gazebo can resolve repository-provided models and their assets.
    - The resource path should include the installed or source models root, for example the parent directory that contains `rock_5/`.
- Asset resolution:
    - Nested assets referenced from `model.sdf`, such as meshes, material scripts, albedo maps, normal maps, and roughness maps, must resolve in the runtime environment as well as in static world loading.
    - Prefer relative paths inside a model package for assets that live under the same model directory. Use `model://...` URIs only when the model is expected to be discoverable through Gazebo resource paths.
- Post-spawn verification:
    - Scenario drivers should verify the spawned entity exists in Gazebo after insertion, for example with a world-side query such as `gz model --list`.
    - The scenario should only report physical obstacle insertion success after both the spawn command and entity verification succeed.
- Observability:
    - Log the resolved model file path, spawn command, effective Gazebo resource-path environment, and the post-spawn verification result.
    - If the spawn command succeeds but Gazebo cannot verify the entity, record that as an observability or injection failure and fall back only if the scenario design allows it.

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
- If runtime spawning is used for uncertainty injection, prefer repository models whose assets have already been validated in Docker with the same resource-path environment used by the scenario driver.
- Before adding or spawning an obstacle for a scenario, verify that the world does not already contain an equivalent obstacle or hazard serving the same evaluation purpose.

## Non-Negotiables
- The Gazebo world configured gravity and lighting are matching the conditions in Mars and should not be changed at any point of the implementation.
- The existing sensor suite for the rover in the project should not be changed at any point of the development or runtime testing.
- Sensor-related properties such as topics, pose, or scan angles should not be changed at any point of the development or runtime testing.
- Any other sensor-related properties being using in the autonomy specification source code, which is under test, should not be changed at any point of the development or runtime testing.

## Code Style and Guidelines
- For uncertainty injection in the simulation environment, existing models in the spacetry_models are prefered since they are more realistic. 
- When using existing Gazebo models from `spacetry_models`, inspect the model SDF for nested asset references before assuming the model is runtime-spawn safe.
- For additional details about Gazebo Harmonic features and good practices can be found in the official documentation in: https://gazebosim.org/docs/harmonic/getstarted/. 
- Follow additional instructions in `AGENTS.md` files from project-wide and package-specific (in the `src/` sub-folders).
- In case of doubt, conflicting or missing information, ask clarification from the user.
