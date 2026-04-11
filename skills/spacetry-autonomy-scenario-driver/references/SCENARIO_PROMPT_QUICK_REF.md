# Quick Scenario Prompt Reference

Compact reference for using the scenario prompt template consistently.

For the parametrizable template, see [SCENARIO_PROMPT_TEMPLATE.md](../assets/SCENARIO_PROMPT_TEMPLATE.md).

---

## Quick Rules

- Keep mission description focused on mission intent, environment, and capabilities.
- Fill every placeholder you keep. Delete lines that are not relevant to the scenario.
---

## Reference Scenario Examples (Template-Conformant)

Use these as parallel examples for different uncertainty families and multi-uncertainty scenarios. Keep the shared project context and adapt the evaluation target, uncertainty injection, challenge, and metrics to the scenario you want to evaluate.

### Example 1: Obstacle / Environment

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
    - Mission planning: spacetry_mission/MISSION.md
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
      - Autonomy assessment: PASS if the rover detects, replans, and continues safely; DEGRADED if it recovers unsafely or too slowly; FAIL if it deadlocks, collides, abandons the objective, or runs out of battery

## Objective

Design, implement, and execute a **Autonomy Test Scenario Driver Software Component** that:

1. **Injects Uncertainty**:
   - Uncertainty location: environment
   - Uncertainty emerging time: runtime
   - Fault subject: route segment between dock_pad_01 and science_rock_01
   - Fault manifestation: obstacle insertion
   - Fault trigger timing: runtime

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
- Recovery rate (ms): Time from the first avoidance maneuver, after obstacle detection, to resumed collision-free progress
- Obstacle detection latency (ms): Time from obstacle injection to the first attributable obstacle evidence on the autonomy-facing perception interface used by the autonomy stack
- Detour distance (m): Extra path length traveled relative to the nominal route after replanning around the obstacle
- Route deviation (m): Maximum or average lateral distance between the full executed path and the nominal planned path; if needed, add a separate post-injection route-deviation metric
```

### Example 2: Battery / Resource

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
    - Mission planning: spacetry_mission/MISSION.md
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
      - Autonomy assessment: PASS if the rover preserves energy-aware behavior and recovers safely; DEGRADED if it reacts too late or too conservatively; FAIL if it ignores low-battery risk, stalls, or cannot complete a safe return

## Objective

Design, implement, and execute a **Autonomy Test Scenario Driver Software Component** that:

1. **Injects Uncertainty**:
   - Uncertainty location: resources
   - Uncertainty emerging time: runtime
   - Fault subject: rover battery resources
   - Fault manifestation: accelerated drain or reduced available state of charge
   - Fault trigger timing: runtime

2. **Tests Autonomous Adaptation**:
   - Can the rover recognize that battery resources have become insufficient for the remaining mission path?
   - Does the autonomy stack switch from nominal traversal to a safe energy-aware behavior without deadlock?
   - Can the rover preserve safety while deciding whether to continue, pause, or return to the outpost?

3. **Challenges Safety and Goal Viability**:
   - Force a tradeoff between goal completion and safe battery preservation
   - Verify that MR_009 related behavior remains consistent with the injected battery stress
   - Assess whether mission continuation decisions remain safe and explainable

## Additional Mission-Specific Metrics to Consider

- Safety preservation (boolean per constraint): Boolean status for each relevant monitor or safety condition, for example `MR_009=true`, `MR_011=true`
- Goal viability (boolean per goal): Boolean status for each mission objective, for example `science_rock_01_reached=true`, `safe_return_completed=true`
- Low-battery response latency (ms): Time from battery stress injection to the first confirmed energy-aware control or mission change
- Battery reserve margin (%): Lowest remaining state of charge reached during recovery or return behavior
- Time to safe-return decision (ms): Time from injected resource stress to a stable decision to continue or return
- Mission truncation point (m): Distance traveled toward the goal before the autonomy stack abandons or defers the objective
- Route deviation (m): Maximum or average lateral distance between the full executed path and the nominal planned path; if needed, add a separate post-injection route-deviation metric
```

### Example 3: Sensor / Perception

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
    - Mission planning: spacetry_mission/MISSION.md
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
      - Autonomy assessment: PASS if the rover tolerates sensing degradation and continues safely; DEGRADED if navigation becomes overly slow or unstable; FAIL if it collides, deadlocks, or abandons the mission without recovery logic

## Objective

Design, implement, and execute a **Autonomy Test Scenario Driver Software Component** that:

1. **Injects Uncertainty**:
   - Uncertainty location: managed system sensing
   - Uncertainty emerging time: runtime
   - Fault subject: rover perception or scan-derived obstacle classification
   - Fault manifestation: noisy, degrading, or intermittent sensor quality
   - Fault trigger timing: runtime

2. **Tests Autonomous Adaptation**:
   - Can the rover continue toward the goal under degraded perception without unsafe oscillation or deadlock?
   - Does the behavior tree maintain coherent navigation decisions when obstacle evidence becomes intermittent or noisy?
   - Can the rover recover normal progress after the sensor degradation ends or stabilizes?

3. **Challenges Safety and Goal Viability**:
   - Introduce a perception degradation that affects obstacle interpretation while the rover is en route
   - Force a tradeoff between nominal speed and cautious behavior under reduced sensing confidence
   - Verify that monitor constraints remain preserved while perception quality is degraded

## Additional Mission-Specific Metrics to Consider

- Safety preservation (boolean per constraint): Boolean status for each relevant monitor or safety condition, for example `MR_009=true`, `MR_011=true`, `collision_with_obstacle=false`
- Goal viability (boolean per goal): Boolean status for each mission objective, for example `science_rock_01_reached=true`, `mission_deadline_met=true`
- Degraded-sensing response latency (ms): Time from perception fault injection to the first stable navigation adaptation
- False obstacle rate (count or %): Rate at which degraded perception produces obstacle classifications unsupported by world state
- Missed-detection rate (count or %): Rate at which relevant hazards are not detected during the degradation window
- Recovery time (ms): Time from restoration of perception quality to stable forward progress on the mission route
- Route deviation (m): Maximum or average lateral distance between the full executed path and the nominal planned path; if needed, add a separate post-injection route-deviation metric
```

### Example 4: Composite / Obstacle + Sensing Degradation

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
    - Mission planning: spacetry_mission/MISSION.md
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
      - Autonomy assessment: PASS if the rover detects, adapts, and continues safely under coupled obstacle and sensing stress; DEGRADED if recovery is overly slow or unstable; FAIL if it collides, deadlocks, or abandons the mission

## Objective

Design, implement, and execute a **Autonomy Test Scenario Driver Software Component** that:

1. **Defines the Evaluation Target**:
   - Primary evaluation target: navigation autonomy under runtime obstacle blocking
   - Secondary injected uncertainties: degraded perception quality affecting obstacle interpretation
   - [Optional] Interaction hypothesis: degraded sensing delays or destabilizes obstacle avoidance and route recovery

2. **Injects Uncertainty**:
   - Uncertainty locations: environment and managed system sensing
   - Uncertainty emerging time: runtime
   - Fault subjects: nominal route segment to science_rock_01 and rover perception
   - Fault manifestation: inserted rock obstacle plus noisy or degrading obstacle classification
   - Fault trigger timing: runtime

3. **Tests Autonomous Adaptation**:
   - Can the rover still detect and avoid a newly introduced obstacle when perception quality is reduced?
   - Does the behavior tree remain stable under simultaneous obstacle pressure and degraded sensing?
   - Can the rover recover forward progress without unsafe oscillation or collision?

4. **Challenges Safety and Goal Viability**:
   - Insert a blocking rock on the nominal route while degrading obstacle interpretation
   - Force a tradeoff between cautious behavior and timely goal progress
   - Verify that monitor constraints remain preserved during the multi-uncertainty disturbance

## Additional Mission-Specific Metrics to Consider

- Safety preservation (boolean per constraint): Boolean status for each relevant monitor or safety condition, for example `MR_009=true`, `MR_011=true`, `collision_with_obstacle=false`
- Goal viability (boolean per goal): Boolean status for each mission objective, for example `science_rock_01_reached=true`, `mission_deadline_met=true`
- Obstacle detection latency (ms): Time from obstacle injection to the first attributable obstacle evidence available to the autonomy stack under degraded sensing
- False obstacle rate (count or %): Rate of unsupported obstacle classifications during the sensing degradation window
- Recovery rate (ms): Time from the first avoidance maneuver, after obstacle detection, to resumed collision-free progress
- Route deviation (m): Maximum or average lateral distance between the full executed path and the nominal planned path; if needed, add a separate post-injection route-deviation metric
```

### Example 5: Composite / Terrain Change + Battery Drain

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
    - Mission planning: spacetry_mission/MISSION.md
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
      - Autonomy assessment: PASS if the rover handles terrain-induced energy stress safely; DEGRADED if it reacts too late or becomes overly conservative; FAIL if it violates safety, depletes battery unsafely, or abandons recovery logic

## Objective

Design, implement, and execute a **Autonomy Test Scenario Driver Software Component** that:

1. **Defines the Evaluation Target**:
   - Primary evaluation target: energy-aware mission adaptation under increased traversal cost
   - Secondary injected uncertainties: runtime battery drain escalation
   - [Optional] Interaction hypothesis: harsher terrain increases traversal effort and amplifies the safety impact of battery loss

2. **Injects Uncertainty**:
   - Uncertainty locations: environment and resources
   - Uncertainty emerging time: runtime
   - Fault subjects: simulation terrain conditions and rover battery resources
   - Fault manifestation: terrain resistance increase plus accelerated battery drain
   - Fault trigger timing: runtime

3. **Tests Autonomous Adaptation**:
   - Can the rover recognize that harsher terrain and battery loss together threaten safe mission completion?
   - Does the autonomy stack transition to a safer energy-aware behavior before violating battery constraints?
   - Can the rover preserve safety while deciding whether to continue, slow down, pause, or return?

4. **Challenges Safety and Goal Viability**:
   - Increase terrain difficulty along the active route while accelerating battery depletion
   - Force a tradeoff between route completion and safe energy preservation
   - Verify that low-battery related monitor behavior remains coherent under the combined disturbance

## Additional Mission-Specific Metrics to Consider

- Safety preservation (boolean per constraint): Boolean status for each relevant monitor or safety condition, for example `MR_009=true`, `MR_011=true`
- Goal viability (boolean per goal): Boolean status for each mission objective, for example `science_rock_01_reached=true`, `safe_return_completed=true`
- Low-battery response latency (ms): Time from terrain and battery stress to the first confirmed energy-aware adaptation
- Battery reserve margin (%): Lowest remaining state of charge during the stressed traversal or recovery
- Terrain-induced speed loss (m/s): Reduction in sustained forward speed after the terrain change is applied
- Time to safe-return decision (ms): Time from multi-uncertainty stress injection to a stable continue-or-return decision
```

For detailed guidance on measurement definitions and attribution rules, see [Scenario_Contract.md](./Scenario_Contract.md) and [Observability_and_Attribution.md](./Observability_and_Attribution.md). For component specification and behavior configuration, see [Component Specification](../SKILL.md#component-specification) and [Behavior Configuration](../SKILL.md#behavior-configuration).

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
- [ ] Detection-latency metrics are defined on the autonomy-consumed perception interface, with any raw-sensor latency tracked separately if needed

---

## Template Adaptation Guide

Use this table to customize the template for your specific scenario:

| Aspect | Customize | Example |
|--------|-----------|---------|
| **Uncertainty Type** | Sensor degradation, dynamic obstacles, resource constraints, environmental change | "LIDAR fails at 50% confidence" |
| **Evaluation Target** | One primary target plus optional secondary injected uncertainties | "Primary: obstacle avoidance, Secondary: LiDAR degradation" |
| **Interaction Hypothesis** | Optional reason why multiple injected uncertainties may interact | "Reduced sensing delays obstacle avoidance" |
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
