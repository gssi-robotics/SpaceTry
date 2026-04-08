# Scenario Prompt Template

Parametrizable template for autonomous test scenario driver prompts. Fill in placeholders with scenario-specific values.

---

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

iii) **Monitors**:
    Location: {MONITORS_PATH}
    Description: ROS 2 package with the monitors for safety constraints
    - Safety Constraints:
      - {CONSTRAINT_ID}: {SAFETY_CONSTRAINT_DESCRIPTION}

iv) **Mission Goals Description**:
     Location: {MISSION_GOAL_FILE}
     Description: {MISSION_DESCRIPTION}
     - Outcome assessment:
      - Goal status: {GOAL_STATUS_RULE}
      - Safety status: {SAFETY_STATUS_RULE}
      - Autonomy assessment: {PASS_DEGRADED_FAIL_RULE}

## Objective

Design, implement, and execute a **Autonomy Test Scenario Driver Software Component** that:

1. **Injects Uncertainty**:
   - Autonomy aspect: {AUTONOMY_ASPECT}
   - Uncertainty type: {UNCERTAINTY_TYPE}
   - Uncertainty location: {UNCERTAINTY_LOCATION}
   - Uncertainty nature: {UNCERTAINTY_NATURE}
   - Fault subject: {FAULT_SUBJECT}
   - Fault attribute: {FAULT_ATTRIBUTE}
   - Manifestation: {MANIFESTATION}
   - Space domain: {SPACE_DOMAIN_OR_NA}
   - Time domain: {TIME_DOMAIN_OR_NA}
   - Trigger: {TRIGGER_CONDITION}

2. **Tests Autonomous Adaptation**:
   - {ADAPTATION_TEST_1}
   - {ADAPTATION_TEST_2}
   - {ADAPTATION_TEST_3}

3. **Challenges Safety and Goal Viability**:
   - {SAFETY_CHALLENGE_1}
   - {SAFETY_CHALLENGE_2}
   - {GOAL_VIABILITY_CHALLENGE_1}

## Uncertainty Injection Plan

- Injection pattern: {INJECTION_PATTERN}
- Fill-in template:
  - Inject/change {TARGET} from {INITIAL_STATE} to {FINAL_STATE} at/when {INJECTION_POINT}
  - Test: {PRIMARY_ADAPTATION_EXPECTATION}
  - Measure: {METRICS_WITH_UNITS}
  - Injection timing: {INJECTION_TIMING}
  - Intensity strategy: {INTENSITY_STRATEGY}

Choose the closest pattern and fill in the blanks:
- Sensor degradation: `Inject {SENSOR} confidence loss from {START}% to {END}% over {DURATION}`
- Dynamic obstacles: `Spawn {OBSTACLE_TYPE} at {LOCATION} when rover is {DISTANCE_OR_STATE} from {REFERENCE_POINT}`
- Power constraints: `Accelerate battery drain from {BASELINE_RATE} to {NEW_RATE} during {ACTIVITY}`
- Environmental changes: `Change {PARAMETER} from {INITIAL} to {FINAL} at {TIMING}`

## Additional Mission-Specific Metrics to Consider

- {METRIC_NAME} ({UNIT_OR_BOOLEAN}): {METRIC_DESCRIPTION}
- {METRIC_NAME} ({UNIT_OR_BOOLEAN}): {METRIC_DESCRIPTION}
- {METRIC_NAME} ({UNIT_OR_BOOLEAN}): {METRIC_DESCRIPTION}
```
---

For a completed example that matches this template, see [SCENARIO_PROMPT_QUICK_REF.md](../references/SCENARIO_PROMPT_QUICK_REF.md).

**Last Updated:** April 8, 2026  
**Applies to:** ROS 2 Jazzy, Gazebo Harmonic, SpaceTry Testbed
