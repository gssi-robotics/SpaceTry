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

1. **Defines the Evaluation Target**:
   - Primary evaluation target: {PRIMARY_UNCERTAINTY_FAMILY_OR_AUTONOMY_CONCERN}
   - Secondary injected uncertainties: {ZERO_OR_MORE_SECONDARY_UNCERTAINTIES}
   - Note: secondary injected uncertainties may be related or unrelated to the primary evaluation target
   - [Optional] Interaction hypothesis: {WHY_THE_PRIMARY_AND_SECONDARY_UNCERTAINTIES_INTERACT}

2. **Injects Uncertainty**:
   - Uncertainty locations: {PRIMARY_AND_OPTIONAL_SECONDARY_LOCATIONS}
   - Uncertainty emerging time: {RUNTIME_OR_DESIGN-TIME}
   - Fault subjects: {PRIMARY_AND_OPTIONAL_SECONDARY_FAULT_SUBJECTS}
   - Fault Manifestation: {FIXED-VALUE_OR_NOISY_OR_DEGRADING_OR_ANY}
   - Fault Trigger Timming: {TRANSIENT_OR_PERMANENT_OR_INTERMITTENT_OR_ANY}

3. **Tests Autonomous Adaptation**:
   - {ADAPTATION_TEST_1}
   - {ADAPTATION_TEST_2}
   - {ADAPTATION_TEST_3}

4. **Challenges Safety and Goal Viability**:
   - {SAFETY_CHALLENGE_1}
   - {SAFETY_CHALLENGE_2}
   - {GOAL_VIABILITY_CHALLENGE_1}

## [Optional] Uncertainty Injection Plan 

Choose one or more patterns that match the primary target and any secondary injected uncertainties:
- Sensor degradation: `Inject {SENSOR} confidence loss from {START}% to {END}% over {DURATION}`
- Dynamic obstacles: `Spawn {OBSTACLE_TYPE} at {LOCATION} when rover is {DISTANCE_OR_STATE} from {REFERENCE_POINT}`
- Power constraints: `Accelerate battery drain from {BASELINE_RATE} to {NEW_RATE} during {ACTIVITY}`
- Environmental changes: `Change {PARAMETER} from {INITIAL} to {FINAL} at {TIMING}`

## [Optional] Additional Mission-Specific Metrics to Consider

- {METRIC_NAME} ({UNIT_OR_BOOLEAN}): {METRIC_DESCRIPTION}
- {METRIC_NAME} ({UNIT_OR_BOOLEAN}): {METRIC_DESCRIPTION}
- {METRIC_NAME} ({UNIT_OR_BOOLEAN}): {METRIC_DESCRIPTION}
```
---

For a completed example that matches this template, see [SCENARIO_PROMPT_QUICK_REF.md](../references/SCENARIO_PROMPT_QUICK_REF.md).

**Last Updated:** April 8, 2026  
**Applies to:** ROS 2 Jazzy, Gazebo Harmonic, SpaceTry Testbed
