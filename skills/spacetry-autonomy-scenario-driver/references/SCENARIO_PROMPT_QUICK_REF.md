# Quick Scenario Prompt Reference

Compact reference for using the scenario prompt template consistently.

For the parametrizable template, see [SCENARIO_PROMPT_TEMPLATE.md](../assets/SCENARIO_PROMPT_TEMPLATE.md).

---

## Quick Rules

- Keep mission description focused on mission intent, environment, and capabilities.
- Fill every placeholder you keep. Delete lines that are not relevant to the scenario.
---

## Reference Scenario Example (Template-Conformant)

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
    - Mission planning: spacetry_mission
    - Battery manager: spacetry_battery
    - Dependency packages: ../deps/spacetry.repos

iii) **Monitors**:
    Location: src/spacetry_monitors
    Description: ROS 2 package with the monitors for safety constraints
    - Safety Constraints:
      - MR_009_RETURN_TO_OUTPOST_ON_LOW_BATTERY: Ensures rover returns to outpost when battery state of charge drops below threshold. Monitors `/battery/soc` and `/battery/near_outpost` topics to trigger handler when low battery detected and rover is not near outpost.
      - MR_011_MAINTAIN_SPEED_WHEN_FULL_BATTERY: Ensures rover maintains full linear velocity when battery is fully charged. Monitors `/battery/soc` and `/cmd_vel` topics to trigger handler when battery is full but linear velocity command falls below full speed threshold.

iv) **Mission Goals Description**:
     Location: REF_SCENARIO.md
     Description: Navigate in the open terrain to reach mission target waypoint while preserving rover safety.
    - Outcome assessment:
      - Goal status: PASS if science_rock_01 is reached before timeout; otherwise FAIL
      - Safety status: PASS if no collision occurs and monitor constraints stay preserved; otherwise DEGRADED or FAIL depending on severity
      - Autonomy assessment: PASS if the rover detects, replans, and continues safely; DEGRADED if it recovers unsafely or too slowly; FAIL if it deadlocks, collides, or abandons the objective

## Objective

Design, implement, and execute a **Autonomy Test Scenario Driver Software Component** that:

1. **Injects Uncertainty**:
   - Autonomy aspect: behavior flexibility
   - Uncertainty type: dynamic obstacles
   - Uncertainty location: environment
   - Uncertainty nature: variability
   - Fault subject: traversable path near mission waypoint
   - Fault attribute: obstacle occupancy along nominal route
   - Manifestation: intermittent
   - Space domain: contiguous
   - Time domain: intermittent

2. **Tests Autonomous Adaptation**:
   - Can the rover detect a newly introduced obstacle on the nominal route?
   - Does the behavior tree transition to replanning or avoidance actions without stalling?
   - Can the rover continue toward the waypoint after the route is blocked?

3. **Challenges Safety and Goal Viability**:
   - Insert a rock obstacle directly on the preferred route to science_rock_01
   - Force a tradeoff between shortest path completion and collision avoidance
   - Verify that monitor constraints remain preserved during replanning

## Additional Mission-Specific Metrics to Consider

- Safety preservation (boolean per constraint): Boolean status for each relevant monitor or safety condition, for example `MR_009=true`, `MR_011=true`, `collision_with_dynamic_obstacle=false`
- Goal viability (boolean per goal): Boolean status for each mission objective, for example `science_rock_01_reached=true`, `mission_deadline_met=true`
- Recovery rate (ms): Time from replanning activation to stable progress on a collision-free route
- Obstacle detection latency (ms): Time from obstacle injection to the first confirmed obstacle detection by the autonomy stack
- Detour distance (m): Extra path length traveled relative to the nominal route after replanning around the obstacle
- Route deviation (m): Maximum or average lateral distance between the executed path and the nominal planned path
```

For detailed guidance on measurement definitions, component specification, and behavior configuration, see [SKILL.md](../SKILL.md#measurement-focus), [Component Specification](../SKILL.md#component-specification), and [Behavior Configuration](../SKILL.md#behavior-configuration) sections.

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
| **Additional Metrics** | Mission-specific metrics with units or boolean status | "Obstacle detection latency (ms)" |
| **Intensity Increase** | Gradual, sudden, cascading | "LIDAR quality: 100% → 80% → 50% → 0% over 10 minutes" |
| **Outcome Assessment** | PASS, DEGRADED, FAIL rules tied to goal and safety evidence | "Goal completed + 0 safety violations = PASS" |

---

## Related Files

- [AGENTS.md](../../../AGENTS.md) - Project-wide autonomy evaluation framework
- [.AGENTS.md.template](../../../.AGENTS.md.template) - Package-specific template with autonomy sections
- [spacetry_mission/MISSION.md](../../../src/spacetry_mission/MISSION.md) - Mission planning details
---

**See Also:** [SCENARIO_PROMPT_TEMPLATE.md](../assets/SCENARIO_PROMPT_TEMPLATE.md) for the parametrizable template.

**Last Updated:** April 8, 2026
