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

**See Also:** [SCENARIO_PROMPT_TEMPLATE.md](./SCENARIO_PROMPT_TEMPLATE.md) for detailed examples and usage guidance.

