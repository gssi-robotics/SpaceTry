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

**Last Updated:** March 27, 2026  
**Applies to:** ROS 2 Jazzy, Gazebo Harmonic, SpaceTry Curiosity rover

