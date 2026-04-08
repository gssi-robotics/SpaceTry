# SpaceTry Project - Agent Instructions

This file defines rules and conventions for LLM agents (Copilot, Codex, Claude Code, etc.) interacting with the SpaceTry robotics project. In case of doubts or missing information, request clarification or input from the user.

## Hierarchy of Rules

Rules are organized hierarchically. When an agent makes changes to content within a package, it MUST follow the most specific applicable rules in this order:

1. **Package-specific rules** - `src/<package>/AGENTS.md` or `src/<package>/.instructions.md`
2. **Project-wide rules** - This file (`AGENTS.md`)
3. **Tool/framework conventions** - ROS2, Gazebo, behavior trees, etc.

When modifying content in a package, always check for package-level `AGENTS.md` or `.instructions.md` files first.

---

## Development Workflow

The project implementation and structure details are in [IMPLEMENTATION.md](docs/IMPLEMENTATION.md).

### 1. ROS 2 Packages Changes:
The rover ROS 2 Packages are:
- `src/spacetry_battery/`
- `src/spacetry_bringup/`
- `src/spacetry_bt/`
- `src/spacetry_mission/`
- `src/spacetry_models/`
- `src/spacetry_monitors/`
- `src/spacetry_perception/`
- `src/spacetry_world/`

Modifications to the listed rover ROS 2 packages above are only allowed during the development stage. After evaluation or during scenario driver generation, changes should be done only if there are bugs detected on the initial implemented behavior. In that case, those changes shall be approved by the user.

When modifying ROS2 packages:

1. **CMakeLists.txt / setup.py** - Declare dependencies, entry points, targets
2. **package.xml** - Metadata, dependencies, build type
3. **Source code** - Follow the package's language conventions (C++, Python)
4. **Configuration files** - In `config/` subdirectories as YAML

After modifying any ROS2 files they should be validated by building and running the simulation:

```bash
 source /opt/ros/spaceros/setup.bash && source /etc/profile && colcon build --merge-install --event-handlers console_direct+
 ```

### 2. Running the Simulation

The project uses Docker Compose to orchestrate the simulation environment:

**Terminal: Start the services and launch the rover**

```bash
   docker exec -it docker-spacetry-1 bash -lc 'source /opt/ros/spaceros/setup.bash && source /ws/install/setup.bash && ros2 launch spacetry_bringup spacetry_curiosity_outpost.launch.py'
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

When running commands in containers, these are pre-set via the [docker compose file](docker/docker-compose.yaml):
- `ROS_DOMAIN_ID=0` - DDS discovery domain
- `RMW_IMPLEMENTATION` - Middleware layer
- `ROS_DISTRO=jazzy` - ROS2 version
- `PYTHONUNBUFFERED=1` - Real-time output

Do not override these unless specifically needed.

---

## Behavior Trees

Located in `src/spacetry_bt/trees/`:
- XML format using BehaviorTree.CPP library
- Modify only after understanding the node definitions
- Test changes by relaunching the rover

## Gazebo Models

Located in `src/spacetry_models/models/`:
- Each model has `model.config` and `model.sdf` (XML/SDF format)
- Modify carefully - affects physics and visualization
- Models auto-load in `spacetry_world/worlds/mars_outpost.sdf`

---

## Autonomy Test Scenario Generation and Evaluation Workflow

Autonomy test scenarios validate the rover's capacity to self-adapt to changing or unforeseen conditions. There is a skill defined in this repository in [spacetry-autonomy-scenario-driver](skills/spacetry-autonomy-scenario-driver) that defines the workflow for the generation of autonomy test scenarios drivers for evaluating the rover's autonomy in simulation. To generate a autonomy test scenario users needs to provide a scenario description. The template [SCENARIO_PROMPT_TEMPLATE.md](skills/spacetry-autonomy-scenario-driver/assets/SCENARIO_PROMPT_TEMPLATE.md) can be used for this task. Guidelines and examples on how to use the template are provided in [SCENARIO_PROMPT_QUICK_REF.md](skills/spacetry-autonomy-scenario-driver/references/SCENARIO_PROMPT_QUICK_REF.md).

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

This file is the canonical source for project-wide rules. Package-specific `AGENTS.md` files should only add package-specific constraints and validation steps, and skills should only add task-specific workflow or output requirements.

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

### Autonomy Changes Checklist

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
- **spacetry_monitors**: safety constraints monitoring. See package-specific rules.
- **spacetry_mission**: Mission planning. Interacts with behaviors and waypoints.
- **spacetry_perception**: Sensor processing. Latency-sensitive, handle carefully.

Package-specific instruction files should avoid repeating the project-wide autonomy checklist unless they add package-specific checks that are not already covered here.

## References

- [ROS2 Jazzy Documentation](https://docs.ros.org/en/jazzy/)
- [Gazebo Documentation](https://gazebosim.org/)
- [BehaviorTree.CPP](https://www.behaviortree.dev/)

---

**Last Updated:** April 8, 2026
**Maintained by:** SpaceTry Team
