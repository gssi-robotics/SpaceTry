# Autonomy Test Scenario Workflow

The workflow consists of:

| Step | Task | Description | Workflow |
|------|------|-------------|----------|
| 1 | **Scenario Prompt Template** | Specify the autonomy under text and mission context and objectives using the prompt template. | User Input |
| 2 | **Scenario Driver Generation** | Use the LLM agent custom Skill to generate and parametrize the evaluation scenario from the prompt. | Automated |
| 3 | **Scenario Driver Parametrization** | Configure and fine-tune scenario parameters before execution. | Automated |
| 4 | **Scenario Driver Execution** | Execute the generated scenario in the simulation environment. | Automated |
| 5 | **Autonomy Evaluation Report** | Analyze the output report from the agent to assess the rover's self-adaptation capabilities in the uncertainty test scenario. | User Input |

## IDE and agent compatibility
This skill has been tested with VS Code and the agents Codex and Copilot. 
Compatibility with additional IDEs and agents are not guaranteed.

## Install the agent scenario driver skill

The recommended setup to install the skill is to load it locally from the repository source linking it to `.agents/skills`.

From the repository root:

```bash
mkdir -p .agents/skills
ln -s ../../skills/spacetry-autonomy-scenario-driver .agents/skills/spacetry-autonomy-scenario-driver
```

Reload your IDE after installing the skill.

For other installation alternatives check the [INSTALL.md](skills/spacetry-autonomy-scenario-driver/INSTALL.md).


## Use the agent to generate and execute the self-adaptive evaluation scenario

Copy the prompt into the agent chat. 

Example of a prompt that follows the template in [SCENARIO_PROMPT_TEMPLATE.md](skills/spacetry-autonomy-scenario-driver/assets/SCENARIO_PROMPT_TEMPLATE.md):

```text
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
    - Dependency packages: deps/spacetry.repos

iii) **Monitors**:
    Location: src/spacetry_monitors
    Description: ROS 2 package with the monitors for safety constraints
    - Safety Constraints:
    - MR_009_RETURN_TO_OUTPOST_ON_LOW_BATTERY: Ensures rover returns to outpost when battery state of charge drops below threshold. Monitors `/battery/soc` and `/battery/near_outpost` topics to trigger handler when low battery detected and rover is not near outpost.
    - MR_011_MAINTAIN_SPEED_WHEN_FULL_BATTERY: Ensures rover maintains full linear velocity when battery is fully charged. Monitors `/battery/soc` and `/cmd_vel` topics to trigger handler when battery is full but linear velocity command falls below full speed threshold.
    - [Optional] Monitor notes:
    - If `/monitor/handlerMR_011_MAINTAIN_SPEED_WHEN_FULL_BATTERY` becomes active, reduced speed may reflect monitor enforcement as well as cautious navigation under degraded sensing.
    - `/monitor/handlerMR_009_RETURN_TO_OUTPOST_ON_LOW_BATTERY` only matters if the run also drifts into battery-related handling.

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

The skill should be automatically used by the agent, even if not explicitly selected or mentioned.

As a result the agent will follow the skill workflow by:
 1. Generate the scenario driver ROS 2 source code. 
 2. Parametrize the generated code via configuration files and validated. 
 3. Execute the generated scenario and adjusted depending on the fine-tunning stage.
 4. Provide the metrics and evaluation report following the formats defined in [src/spacetry_scenario_metrics](src/spacetry_scenario_metrics), once a full run is completed.

An example of the output of the skill usage can be found in [logs/skill_example](logs/skill_example).

A quick reference guide with examples of prompts using the template are available at [SCENARIO_PROMPT_QUICK_REF.md](skills/spacetry-autonomy-scenario-driver/references/SCENARIO_PROMPT_QUICK_REF.md).
