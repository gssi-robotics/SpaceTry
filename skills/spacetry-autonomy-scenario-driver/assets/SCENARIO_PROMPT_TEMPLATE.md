# Test Scenario Prompt Template

This template guides the creation of natural language prompts for autonomous test scenario drivers. Use this to generate or design scenario components that inject uncertainty and challenge the rover's self-adaptation capabilities.

---

## Base Template

```
Given the following components:

i) **Robot Behavior Definition** (Behavior Tree):
   Location: {BT_FILE_PATH}
   Description: This XML file defines the autonomous behavior tree for the rover, specifying:
    - Primary robot behavior via control and action tree nodes

ii) **Implementation Source Code**:
    Location: {SOURCE_CODE_PATH}
    Description: source code with the ROS 2 packages that implement the behavior tree logic and handle real-time decision-making:
    - Perception: {PERCEPTION_MODULE_PATH}
    - Behavior execution: {BT_RUNNER_PATH}
    - Mission planning: {MISSION_PLANNER_PATH}
    - Battery manager: {BATTERY_MANAGER_PATH}
    - Dependency packages: {ROS2_DEPENDENCIES_PATH}

iii) **Mission Description**:
     Location: {MISSION_CONFIG_FILE}
     Description: Natural Language description of the mission goals, safety constraints, and robot capabilities.
     - Success criteria options: Autonomy achieved (adapted & safe), Degraded (adapted, unsafe), or  Failed (not adapted).

iv) **Monitors**:
    Location: {SOURCE_CODE_PATH}
    Description: ROS 2 package with the monitors for safety constraints
    - Safety Constraints: {SAFETY_CONSTRAINTS}

## Objective

Design and implement a **Scenario Driver Software Component** that:

1. **Injects Uncertainty** during robot execution considering:
   - Simulated sensors (perception capability)
   - Dynamic obstacles or other changes in the simulation environment
   - Impose resource constraints
   - Trigger unexpected state changes

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

4. **Measurement Focus**:
   - Adaptation speed (time in milliseconds between the uncertainty was injected and the reaction of the robot to it)
   - Safety preservation (key-value pair with the safety constraints derived from the monitors and their preservation state in boolean) 
   - Goal viability (key-value pair with the goal and a boolean with indication if the goal is viable)
   - Recovery rate (time in milliseconds between the reaction of the robot to the triggered uncertainty and the reaction outcome)

## Component Specification

The driver component should:

- **Monitor** the rover's execution state (position, battery, sensor status)
- **Trigger** uncertainty events at decision-critical moments (e.g., when rover commits to action)
- **Measure** adaptation success (goal completion, safety violations, contingency activations)
- **Adapt injection intensity** based on observed autonomy performance (gradually increase challenge)
- **Log behavior** for post-execution analysis (decision timestamps, fallback activations, constraint violations)

## Component Behavior Configuration:

For each autonomy and safety requirement being evaluated, the driver component should target one behavior of each category below:

1. **Injection Timing**:
   - At decision point of the behavior tree 
   - Mid-action 
   - During contingency

2. **Intensity Increase**:
   - Gradual 
   - Sudden 
   - Cascading


## Uncertainty Injection Templates

1. Sensor Degradation:

   Inject {SENSOR} confidence loss: {START}% → {END}% over {DURATION}
   Test: Can rover adapt perception to {FALLBACK_SENSOR}?
   Measure: Time to fallback, goal completion, collision count

2. Dynamic Obstacles:

      Spawn {OBSTACLE_TYPE} at {LOCATION} when rover is {DISTANCE} away
      Test: Can rover replan route and navigate around obstacle?
      Measure: Route replanning latency, detour distance, deadline miss

3. Power Constraints:

      Accelerate battery drain: baseline {DRAIN_RATE} → accelerated {NEW_RATE} during {ACTIVITY}
      Test: Can rover switch to energy-conserving behaviors and complete mission?
      Measure: Energy reserve margin, mission completion %, behavior transitions

4. Environmental Changes

      Change {PARAMETER} from {INITIAL} to {FINAL} at {TIMING}
      Test: Can rover handle mission objective changes / environment shifts / weather?
      Measure: Replanning time, goal adjustment, recovery success
```

---

**Last Updated:** March 30, 2026  
**Applies to:** ROS 2 Jazzy, Gazebo Harmonic, SpaceTry Testbed

