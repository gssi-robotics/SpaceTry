# Quick Scenario Prompt Template

Use this concise template to rapidly generate natural language prompts for mission scenario drivers.

## One-Liner Format

```
Given the behavior tree at {BT_PATH}, source code in {CODE_PACKAGE}, and mission defined in {MISSION_FILE},
create a scenario driver that injects {UNCERTAINTY_TYPE} to test whether autonomous {ADAPTATION_ASPECT} can
maintain {SUCCESS_CRITERION}.
```

**Example Usage:**
```
Given the behavior tree at src/spacetry_bt/trees/base_bt.xml, source code in src/spacetry_perception,
and mission defined in src/spacetry_mission/config/mission_01.yaml, create a scenario driver that injects
LIDAR degradation to test whether autonomous perception adaptation can maintain obstacle avoidance.
```

---

## Expandable Format (for detailed scenarios)

```
Scenario: {NAME}

Given:
- Behavior Tree: {BT_FILE} 
- Source: {CODE_MODULES}
- Mission: {MISSION_FILE}

Uncertainty: {DESCRIPTION}
- Injection Point: {WHEN}
- Intensity Progression: {HOW_IT_ESCALATES}

Test Objective: Verify autonomous {CAPABILITY} can {ADAPTATION_STRATEGY}

Success Criteria:
- Goal Status: {PASS_CONDITION}
- Safety Status: {CONSTRAINT_STATUS}
- Autonomy Assessment: {PASS/DEGRADED/FAIL}

Metrics to Log: {MEASUREMENTS}
```

**Example Expansion:**
```
Scenario: Progressive LIDAR Degradation

Given:
- Behavior Tree: src/spacetry_bt/trees/base_bt.xml
- Source: src/spacetry_perception, src/spacetry_bt/src/bt_runner.cpp
- Mission: src/spacetry_mission/config/mission_01.yaml (navigate to rock site, sample, return)

Uncertainty: LIDAR confidence drops from 100% → 50% → fail
- Injection Point: After rover detects first waypoint
- Intensity Progression: 0% → 20% degradation every 2 minutes, final failure at T+5min

Test Objective: Verify autonomous perception adaptation can fall back to camera-based navigation

Success Criteria:
- Goal Status: Reached target waypoint despite partial perception loss
- Safety Status: Zero collision/boundary violations
- Autonomy Assessment: PASS (adapted to fallback) or DEGRADED (slow/inefficient) or FAIL (collided/missed goal)

Metrics to Log: LIDAR confidence %, time-to-fallback, camera activation, route deviation, goal ETA
```

---

## Uncertainty Injection Templates

### Sensor Degradation
```
Inject {SENSOR} confidence loss: {START}% → {END}% over {DURATION}
Test: Can rover adapt perception to {FALLBACK_SENSOR}?
Measure: Time to fallback, goal completion, collision count
```

### Dynamic Obstacles
```
Spawn {OBSTACLE_TYPE} at {LOCATION} when rover is {DISTANCE} away
Test: Can rover replan route and navigate around obstacle?
Measure: Route replanning latency, detour distance, deadline miss
```

### Power Constraints
```
Accelerate battery drain: baseline {DRAIN_RATE} → accelerated {NEW_RATE} during {ACTIVITY}
Test: Can rover switch to energy-conserving behaviors and complete mission?
Measure: Energy reserve margin, mission completion %, behavior transitions
```

### Environmental Changes
```
Change {PARAMETER} from {INITIAL} to {FINAL} at {TIMING}
Test: Can rover handle mission objective changes / environment shifts / weather?
Measure: Replanning time, goal adjustment, recovery success
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

## Examples and Guidance

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

**See Also:** [SCENARIO_PROMPT_TEMPLATE.md](./SCENARIO_PROMPT_TEMPLATE.md) for the template.

