# Test Scenario Prompt Template

This template guides the creation of natural language prompts for autonomous test scenario drivers. Use this to generate or design scenario components that inject uncertainty and challenge the rover's self-adaptation capabilities.

---

## Base Template

```
Given the following components:

i) **Robot Behavior Definition** (Behavior Tree):
   Location: {BT_FILE_PATH}
   Description: This XML file defines the autonomous decision tree for the rover, specifying:
   - Primary behaviors (e.g., navigate_to_waypoint, collect_sample, return_to_base)
   - Decision logic (e.g., obstacle_detected → navigate_around)
   - Contingency actions (e.g., low_battery → abort_mission_and_return)
   - Sensor-dependent transitions (e.g., perception_failure → fallback_mode)

ii) **Implementation Source Code**:
    Modules: {SOURCE_CODE_PATHS}
    Key files:
    - Perception nodes: {PERCEPTION_MODULE_PATH}
    - Behavior execution: {BT_RUNNER_PATH}
    - Mission planning: {MISSION_PLANNER_PATH}
    - Battery manager: {BATTERY_MANAGER_PATH}
    These implement the behavior tree logic and handle real-time decision-making.

iii) **Mission Description** (Natural Language):
     Location: {MISSION_CONFIG_FILE}
     Objectives: {PRIMARY_GOALS}
     Waypoints: {WAYPOINT_LIST}
     Success criteria: {SUCCESS_CONDITIONS}
     Constraints: {SAFETY_CONSTRAINTS}

---

## Objective

Design a **Scenario Driver Software Component** that:

1. **Injects Uncertainty** during robot execution:
   - Simulate sensor degradation or failure (losing perception capability)
   - Introduce dynamic obstacles or environmental changes (unforeseen hazards)
   - Impose resource constraints (e.g., battery drain acceleration, actuator failures)
   - Trigger unexpected state changes (e.g., switch mission objectives mid-execution)

2. **Tests Autonomous Adaptation**:
   - Can the rover adapt perception strategies when sensors degrade?
   - Does the behavior tree transition to contingency actions appropriately?
   - Does the rover maintain safety constraints while adapting to uncertainty?
   - Can mission planning adjust goals when resources become limited?

3. **Challenges Safety and Goal Viability**:
   - Insert conditions that could violate safety constraints (e.g., approach hazards, collide with obstacles)
   - Create tradeoffs between goal completion and safety (e.g., reach target vs. avoid collision)
   - Monitor whether the rover's autonomy resolves conflicts correctly or fails catastrophically
   - Log which autonomy aspects degrade first under uncertainty accumulation

---

## Component Specification

The driver component should:

- **Monitor** the rover's execution state (position, battery, sensor status)
- **Trigger** uncertainty events at decision-critical moments (e.g., when rover commits to action)
- **Measure** adaptation success (goal completion, safety violations, contingency activations)
- **Adapt injection intensity** based on observed autonomy performance (gradually increase challenge)
- **Log behavior** for post-execution analysis (decision timestamps, fallback activations, constraint violations)

---

## Instantiation Examples

### Example 1: Sensor Degradation Scenario
```
Given:
i) BT: src/spacetry_bt/trees/base_bt.xml (contains perception → navigation logic)
ii) Code: src/spacetry_perception/ (LIDAR, camera nodes)
iii) Mission: src/spacetry_mission/config/mission_01.yaml (navigate to rock site, sample, return)

Create a driver that:
- Simulates progressive LIDAR degradation over 5 minutes (start at 100% → drop to 50% → fail)
- Monitors: Does rover switch from LIDAR-guided navigation to camera-guided fallback?
- Inject at decision point: Force rover to choose action while perception is partially degraded
- Measure: Time to adapt, goal progress, safety violations (collisions, boundary crossings)
```

### Example 2: Dynamic Obstacle Scenario
```
Given:
i) BT: src/spacetry_bt/trees/base_bt.xml (contains obstacle_detection → navigate_around)
ii) Code: src/spacetry_perception/ + src/spacetry_world/worlds/mars_outpost.sdf
iii) Mission: src/spacetry_mission/config/mission_01.yaml (traverse open terrain)

Create a driver that:
- Places dynamic obstacles (moving rocks, sliding terrain) at waypoints
- Monitors: Does rover detect, replan, and navigate around obstacles?
- Inject at decision point: Block primary route when rover commits to movement
- Measure: Route replanning latency, detour distance, goal deadline violation
```

### Example 3: Power Constraint Scenario
```
Given:
i) BT: src/spacetry_bt/trees/base_bt.xml (contains power_check → mission_adjust logic)
ii) Code: src/spacetry_battery/battery_manager_node.py
iii) Mission: src/spacetry_mission/config/mission_01.yaml (multi-waypoint survey)

Create a driver that:
- Accelerates battery drain during high-power operations (movement, sampling)
- Monitors: Does rover switch to energy-conserving behaviors (slower motion, sensor throttling)?
- Inject at decision point: Force choice between completing mission segment vs. reserving energy
- Measure: Graceful degradation (mission completion %), energy awareness (reserve margin), constraint violations
```

---

## Template Adaptation Guide

Use this table to customize the template for your specific scenario:

| Aspect | Customize | Example |
|--------|-----------|---------|
| **Uncertainty Type** | Sensor degradation, dynamic obstacles, resource constraints, environmental change | "LIDAR fails at 50% confidence" |
| **Injection Timing** | At decision point, mid-action, during contingency | "When rover commits to navigate_to (waypoint 3)" |
| **Measurement Focus** | Adaptation speed, safety preservation, goal viability, recovery rate | "Time to activate obstacle_avoidance behavior" |
| **Intensity Increase** | Gradual, sudden, cascading | "LIDAR quality: 100% → 80% → 50% → 0% over 10 minutes" |
| **Success Criteria** | Autonomy achieved (adapted & safe), degraded (adapted, unsafe), failed (not adapted) | "Goal completed + 0 safety violations = SUCCESS" |

---

## Usage in Scenario Driver Code

Apply this template when generating or creating scenario driver components:

1. **Parse the template** - Extract BT location, code paths, mission file
2. **Identify uncertainty injection points** - Read BT to find decision nodes
3. **Create injection logic** - Implement event triggers (sensor degrade, obstacle spawn, power drain)
4. **Instrument monitoring** - Subscribe to relevant ROS2 topics for state tracking
5. **Measure and log** - Record autonomy metrics (recovery rate, goal completion, safety violations)
6. **Report results** - Generate scenario evaluation with autonomy impact assessment

---

## Integration with Test Scenario Workflow

This template supports the [Test Scenario Generation Workflow](./AGENTS.md#test-scenario-generation-workflow) by:

- **Step 1** (Define Autonomy Test Context) → Use template to clarify scenario objectives
- **Step 2** (Build and Prepare) → Template specifies code modules to build
- **Step 3** (Launch Scenario) → Driver component executes uncertainty injection during launch
- **Step 4** (Execute & Monitor) → Measure autonomy metrics defined in template
- **Step 5** (Analyze Results) → Assess which autonomy aspects succeeded/failed

---

## Related Files

- [AGENTS.md](./AGENTS.md) - Project-wide autonomy evaluation framework
- [.AGENTS.md.template](../.AGENTS.md.template) - Package-specific template with autonomy sections
- [spacetry_mission/MISSION.md](../src/spacetry_mission/MISSION.md) - Mission planning details
- [spacetry_bt/README](../src/spacetry_bt/README.md) - Behavior tree execution guide

---

**Last Updated:** March 27, 2026  
**Applies to:** ROS 2 Jazzy, Gazebo Harmonic, SpaceTry Curiosity rover

