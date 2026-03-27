# SpaceTry Project - Agent Instructions

This file defines rules and conventions for LLM agents (Copilot, Codex, etc.) interacting with the SpaceTry robotics project.

## Hierarchy of Rules

Rules are organized hierarchically. When an agent makes changes to content within a package, it MUST follow the most specific applicable rules in this order:

1. **Package-specific rules** - `src/<package>/AGENTS.md` or `src/<package>/.instructions.md`
2. **Project-wide rules** - This file (`AGENTS.md`)
3. **Tool/framework conventions** - ROS2, Gazebo, behavior trees, etc.

When modifying content in a package, always check for package-level `AGENTS.md` or `.instructions.md` files first.

---

## Project Structure Overview

```
spacetry/
├── AGENTS.md (this file - project-wide rules)
├── docker/
│   ├── docker-compose.yaml (defines spacetry service)
│   └── Dockerfile (builds spacetry:dev image)
├── src/
│   ├── spacetry_battery/           - Battery manager node
│   ├── spacetry_bringup/           - Rover launch configurations
│   ├── spacetry_bt/                - Behavior tree runner (C++)
│   ├── spacetry_mission/           - Mission planning and execution
│   ├── spacetry_models/            - Gazebo models (rocks, solar panels, station)
│   ├── spacetry_monitors/          - FRETish monitoring agent
│   ├── spacetry_perception/        - Perception nodes
│   └── spacetry_world/             - Gazebo world configurations
├── scripts/                        - Utility scripts
└── docs/                          - Project documentation
```

---

## Development Workflow

### 1. Running the Simulation

The project uses Docker Compose to orchestrate the simulation environment:

**Terminal 1: Start the services**
```bash
cd docker
docker compose up spacetry
```

**Terminal 2: Launch the rover**
```bash
docker compose exec spacetry bash -lc "ros2 launch spacetry_bringup spacetry_curiosity_outpost.launch.py headless:=0"
```

This starts:
- Gazebo simulation server with the mars_outpost world
- Curiosity rover with all controllers
- ROS2 nodes (battery manager, perception, etc.)

**Important Environment Variables:**
- `ROS_DOMAIN_ID=0` - All containers must match
- `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` - Middleware layer
- `ROS_DISTRO=jazzy` - All code targets ROS 2 Jazzy

---

## Docker Interaction Requirements

**All agents MUST use Docker for any command that:**
- Builds or compiles code (`colcon`, `cmake`, `g++`)
- Runs ROS2 commands (`ros2 launch`, `ros2 run`, `ros2 topic`, etc.)
- Accesses Gazebo or simulation (`gz`, `gazebo` commands)
- Installs/manages ROS2 packages
- Tests code within the simulation environment

### Why Docker?

1. **Consistency** - Exact same environment across all development
2. **Reproducibility** - No host OS dependencies or version conflicts
3. **Safety** - Changes don't affect the host system
4. **Isolation** - Multiple projects can coexist without interference
5. **CI/CD Ready** - Same environment as automated testing

### Standard Docker Commands for Agents

**Build/Compile:**
```bash
docker compose -f docker/docker-compose.yaml exec spacetry colcon build --packages-select <package-name>
```

**Run ROS2 Commands:**
```bash
docker compose -f docker/docker-compose.yaml exec spacetry bash -lc "ros2 <command>"
```

**Launch Rover:**
```bash
docker compose -f docker/docker-compose.yaml exec spacetry bash -lc "ros2 launch spacetry_bringup spacetry_curiosity_outpost.launch.py [args]"
```

**Install Python Packages (in spacetry container):**
```bash
docker compose -f docker/docker-compose.yaml exec spacetry pip install <package>
```

**Interactive Shell:**
```bash
docker compose -f docker/docker-compose.yaml exec spacetry bash -l
```

### When NOT to Use Docker

❌ **Don't** use Docker for:
- Reading files (use native file operations)
- Text/code analysis
- Planning changes
- Querying git history
- Generating code to propose

✅ **Only run in Docker** when:
- Actually executing the code
- Testing code behavior
- Building/compiling
- Running simulations
- Checking ROS2/Gazebo connectivity

### Docker Compose Service Layout

**spacetry service:**
- Main development environment
- Contains ROS2, Gazebo, all simulation nodes
- Runs the rover simulation
- Hosts all build artifacts

### Environment Variables in Docker

When running commands in containers, these are pre-set:
- `ROS_DOMAIN_ID=0` - DDS discovery domain
- `RMW_IMPLEMENTATION` - Middleware layer
- `ROS_DISTRO=jazzy` - ROS2 version
- `PYTHONUNBUFFERED=1` - Real-time output

Do not override these unless specifically needed.

---

## Test Scenario Generation Workflow

Test scenarios validate the rover's autonomous capabilities and capacity to self-adapt to changing or unforeseen conditions. This workflow describes how to generate, configure, and execute test scenarios to evaluate robot autonomy.

### Scenario Components

A test scenario consists of:

1. **World Configuration** - `src/spacetry_world/worlds/mars_outpost.sdf`
   - Terrain complexity and variability
   - Dynamic/static obstacles
   - Environmental conditions (lighting, physics parameters)
   - Unexpected element placement

2. **Rover Configuration** - Launch parameters in `src/spacetry_bringup/`
   - Initial pose and spawn location
   - Sensor calibration state
   - Battery state (nominal, degraded, critical)
   - Autonomous capability parameters

3. **Mission Configuration** - `src/spacetry_mission/config/`
   - Primary objectives and waypoints
   - Tolerance margins for success criteria
   - Fallback behaviors and contingencies
   - Autonomy decision thresholds

4. **Monitoring & Validation** - `src/spacetry_monitors/`
   - Safety constraints (do not violate)
   - Performance thresholds (self-adaptation indicators)
   - Behavior state transitions
   - Deviation detection and logging

### Test Scenario Workflow

For detailed guidance on creating test scenario prompts, see [SCENARIO_PROMPT_TEMPLATE.md](./docs/SCENARIO_PROMPT_TEMPLATE.md).

**Step 1: Define Autonomy Test Context**
```bash
# Specify scenario parameters that test autonomous adaptation
vim scenarios/mission_01.yaml
# Examples:
#   - degraded_sensors: true (test adaptation to sensor loss)
#   - dynamic_obstacles: true (test obstacle avoidance adaptation)
#   - power_constraints: true (test energy-aware behavior)
```

**Step 2: Build and Prepare**
```bash
docker compose exec spacetry colcon build --packages-select spacetry_mission spacetry_world spacetry_bt
```

**Step 3: Launch Scenario with Autonomy Variations**
```bash
docker compose exec spacetry bash -lc \
  "ros2 launch spacetry_bringup spacetry_curiosity_outpost.launch.py \
   headless:=0 spawn_waypoint:=dock_pad_01 battery:=0.5 autonomy_level:=high"
```

**Step 4: Execute Test & Monitor Autonomous Behavior**
- Observe rover decision-making under uncertainty
- Monitor behavior transitions when conditions change
- Track adaptation to obstacles and sensor degradation
- Log autonomy metrics (recovery rate, goal completion, safety violations)

**Step 5: Analyze Autonomy Results**
- Did the rover adapt to unforeseen conditions?
- Were contingency behaviors triggered appropriately?
- Did it maintain safety while pursuing objectives?
- How did autonomy degrade gracefully with resource constraints?

### Autonomy Evaluation Criteria

Test scenarios assess:

- **Perceptual Adaptation** - Does the rover handle sensor degradation? (spacetry_perception)
- **Behavioral Flexibility** - Can behaviors transition when conditions change? (spacetry_bt)
- **Resource-Aware Autonomy** - Does the rover adapt goals to energy constraints? (spacetry_battery)
- **Obstacle Intelligence** - Does it discover and avoid dynamic obstacles? (spacetry_perception + spacetry_bt)
- **Mission Resilience** - Can it recover from failures or replanning? (spacetry_mission)
- **Safety Under Autonomy** - Does self-adaptation never violate safety? (spacetry_monitors)

### Package Impact on Autonomous Capabilities

Different packages contribute to rover autonomy:

- **spacetry_world** - Environmental complexity and uncertainty sources
- **spacetry_bringup** - Sensor/actuator configuration and reliability baseline
- **spacetry_mission** - Mission objectives and adaptation thresholds
- **spacetry_bt** - Autonomous decision trees and contingency behaviors
- **spacetry_perception** - Autonomy through sensing, adaptation to degradation
- **spacetry_battery** - Energy-aware autonomy and resource constraints
- **spacetry_monitors** - Safety constraints on autonomous decisions

When modifying a package, evaluate how it changes the rover's autonomous capabilities and self-adaptation potential.

---

Each `src/*/` directory is a ROS2 package. When modifying:

1. **CMakeLists.txt / setup.py** - Declare dependencies, entry points, targets
2. **package.xml** - Metadata, dependencies, build type
3. **Source code** - Follow the package's language conventions (C++, Python)
4. **Configuration files** - In `config/` subdirectories as YAML

After modifying any ROS2 files:
```bash
docker compose exec spacetry colcon build --packages-select <package-name>
```

### Behavior Trees

Located in `src/spacetry_bt/trees/`:
- XML format using BehaviorTree.CPP library
- Modify only after understanding the node definitions
- Test changes by relaunching the rover

### Gazebo Models

Located in `src/spacetry_models/models/`:
- Each model has `model.config` and `model.sdf` (XML/SDF format)
- Modify carefully - affects physics and visualization
- Models auto-load in `spacetry_world/worlds/mars_outpost.sdf`

---

## Conventions

### Code Style

- **Python**: Follow PEP 8. Use type hints for clarity.
- **C++**: Follow ROS2 C++ style guide. Use camelCase for variables.
- **CMake/Setup files**: Keep consistent with existing patterns
- **Configuration files**: YAML format, clearly commented

### Naming

- ROS2 nodes: `snake_case_node_name`
- ROS2 topics: `/namespace/topic_name`
- ROS2 services: `/namespace/service_name`
- Parameters: `use_sim_time`, `scan_topic`, etc.

### Error Handling

- Include error messages that help debugging
- Log important state transitions
- Use ROS2 logging levels appropriately (INFO, WARN, ERROR)

### Testing

Before proposing changes:
1. Check if the feature/change affects other systems
2. Understand launch dependencies
3. Consider simulation impact (CPU, timing)

---

## Contribution Guidelines for Agents

When making changes to this codebase:

1. **Autonomy Impact First** - Before implementing, assess how changes affect autonomous adaptation:
   - Will the rover adapt better or worse to unforeseen conditions (sensor degradation, dynamic obstacles, power constraints)?
   - Do changes improve perception, behavior flexibility, resource awareness, or mission resilience?
   - Will graceful degradation and contingency fallbacks still work properly?

2. **Read this file first** - Understand project structure, hierarchical rules, and Docker requirements

3. **Check package-specific rules** - Look for AGENTS.md in the package being modified (package rules override project-wide rules)

4. **ALL commands must run in Docker** - ROS2, Gazebo, build, test, or compilation commands must execute in the spacetry container

5. **Test autonomy impact** - Run relevant test scenarios to validate autonomous adaptation:
   - Sensor degradation scenarios (verify perception adaptation)
   - Dynamic obstacle scenarios (verify behavior flexibility)
   - Power constraint scenarios (verify resource-aware autonomy)
   - Monitor recovery rates, goal completion, safety constraint preservation

6. **Preserve existing patterns** - Match the style and structure of surrounding code

7. **Update related files** - If changing launch configs, sensors, behaviors, or missions, update documentation and validate impact on other systems

### Docker is Mandatory

**Every** ROS2 command, compilation, or testing MUST happen in the container. Standard commands:
```bash
# Build
docker compose -f docker/docker-compose.yaml exec spacetry colcon build --packages-select <package>

# Run ROS2
docker compose -f docker/docker-compose.yaml exec spacetry bash -lc "ros2 <command>"

# Launch Rover
docker compose -f docker/docker-compose.yaml exec spacetry bash -lc "ros2 launch spacetry_bringup spacetry_curiosity_outpost.launch.py [args]"
```

### Autonomy Evaluation Checklist

Before submitting changes:
- [ ] Does the change improve, maintain, or degrade autonomous capabilities?
- [ ] Have relevant autonomy test scenarios been run?
- [ ] Are contingency behaviors still available and functional?
- [ ] Do safety constraints hold even with autonomous decision-making?
- [ ] Has graceful degradation been preserved (can rover operate with reduced autonomy)?
- [ ] Is the change documented in the appropriate package's AGENTS.md?

### For Specific Packages

- **spacetry_bringup**: Launch configurations. Changes here affect all subsequent launches.
- **spacetry_bt**: Behavior tree definitions. Complex dependency with rover controllers.
- **spacetry_monitors**: FRETish agent and monitoring. See package-specific rules.
- **spacetry_mission**: Mission planning. Interacts with behaviors and waypoints.
- **spacetry_perception**: Sensor processing. Latency-sensitive, handle carefully.

---

## References

- [ROS2 Jazzy Documentation](https://docs.ros.org/en/jazzy/)
- [Gazebo Documentation](https://gazebosim.org/)
- [BehaviorTree.CPP](https://www.behaviortree.dev/)

---

**Last Updated:** March 26, 2026
**Maintained by:** SpaceTry Team
