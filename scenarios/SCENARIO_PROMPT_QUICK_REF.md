# Quick Scenario Prompt Template

Use this concise template to rapidly generate natural language prompts for mission scenario drivers.

## One-Liner Format

```
Given the behavior tree at src/spacetry_bt/trees/base_bt.xml, source code in src/spacetry_perception and src/spacetry_world,
and mission defined in src/spacetry_mission/MISSION.md, create a scenario driver that injects
uncertainty to test whether autonomous navigation and replanning can maintain obstacle avoidance.
```

---

## Template Instantiation Examples

### Example 1: Basic Benchmark Scenario

```
Given the following components:

i) **Robot Behavior Definition** (Behavior Tree):
   Location: src/spacetry_bt/trees/base_bt.xml
   Description: This XML file defines the autonomous behavior tree for the rover, specifying:
    - Primary robot behavior via control and action tree nodes

ii) **Implementation Source Code**:
    Location: src/
    Description: source code with the ROS 2 packages that implement the behavior tree logic and handle real-time decision-making:
    - Perception: spacetry_perception
    - Behavior execution: spacetry_bt
    - Mission Details: spacetry_mission
    - Battery manager: spacetry_battery
    - Dependency packages: ../deps/spacetry.repos

iii) **Mission Description**:
     Location: src/spacetry_mission/MISSION.md
     Description: Natural Language description of the mission goals, safety constraints, and robot capabilities.
     - Success criteria options: Autonomy achieved (adapted & safe), Degraded (adapted, unsafe), or  Failed (not adapted).
     - Constraints:
        - Constraints: {SAFETY_CONSTRAINTS}

iv) **Monitors**:
    Location: src/spacetry_monitors
    Description: ROS 2 package with the monitors for safety constraints
    - Safety Constraints:
      - MR_009_RETURN_TO_OUTPOST_ON_LOW_BATTERY: Ensures rover returns to outpost when battery state of charge drops below threshold. Monitors `/battery/soc` and `/battery/near_outpost` topics to trigger handler when low battery detected and rover is not near outpost.
      - MR_011_MAINTAIN_SPEED_WHEN_FULL_BATTERY: Ensures rover maintains full linear velocity when battery is fully charged. Monitors `/battery/soc` and `/cmd_vel` topics to trigger handler when battery is full but linear velocity command falls below full speed threshold.

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

## Code Style and Guidelines
- Follow instructions provided by the ROS2 community, available in: https://docs.ros.org/en/rolling/The-ROS2-Project/Contributing/Code-Style-Language-Versions.html
- Follow additional instructions from the AGENTS.md file in the project and sub-folders (packages).
```

**Example Expansion:**
```
Scenario: Dynamic Obstacle Avoidance

Given:
- Behavior Tree: src/spacetry_bt/trees/base_bt.xml
- Source: src/spacetry_perception, src/spacetry_world/worlds/mars_outpost.sdf
- Mission: src/spacetry_mission/config/mission_01.yaml (traverse open terrain)

Uncertainty: Dynamic obstacles (moving rocks, sliding terrain) placed at waypoints
- Injection Point: When rover commits to movement toward waypoint
- Intensity Progression: Single obstacle → Multiple obstacles → Cascading obstacle placement every 30 seconds

Test Objective: Verify autonomous navigation and replanning can detect obstacles and navigate around them

Success Criteria:
- Goal Status: Reached target waypoint despite obstacles blocking primary route
- Safety Status: Zero collision with dynamic obstacles
- Autonomy Assessment: PASS (detected and replanned) or DEGRADED (slow/inefficient replanning) or FAIL (collision/deadline miss)

Metrics to Log: Obstacle detection latency, replanning latency, detour distance, route deviation, deadline violation
```

---

## Scenario Naming Convention

For consistency, name scenarios:
```
scenario_{autonomy_aspect}_{uncertainty_type}_{intensity}
```

**Examples:**
- `scenario_perception_lidar_degradation_gradual`
- `scenario_navigation_dynamic_obstacles_dense`
- `scenario_mission_power_constraints_critical`
- `scenario_safety_cascading_failures_multiple`

---

## Integration Checklist

Before using a scenario prompt to generate or implement a driver:

- [ ] BT path confirmed and readable
- [ ] Source code modules identified and buildable
- [ ] Mission file exists and contains required objectives
- [ ] Uncertainty injection point is at a decision node in BT
- [ ] Success criteria are measurable from ROS2 topics
- [ ] Metrics logging doesn't impact real-time performance
- [ ] Scenario can run in docker compose environment
- [ ] Autonomy aspect being tested is clearly documented

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

## Related Files

- [AGENTS.md](./AGENTS.md) - Project-wide autonomy evaluation framework
- [.AGENTS.md.template](../.AGENTS.md.template) - Package-specific template with autonomy sections
- [spacetry_mission/MISSION.md](../src/spacetry_mission/MISSION.md) - Mission planning details
- [spacetry_bt/README](../src/spacetry_bt/README.md) - Behavior tree execution guide

---

**See Also:** [SCENARIO_PROMPT_TEMPLATE.md](./SCENARIO_PROMPT_TEMPLATE.md) for the template.

