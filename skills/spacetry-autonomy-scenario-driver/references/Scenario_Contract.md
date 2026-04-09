# Scenario Contract

Write the scenario in terms of these fields:

- `autonomy_aspect`: perception, behavior flexibility, resource awareness, mission resilience, obstacle intelligence, or safety
- `uncertainty_location`: environment, model, adaptation functions, goals, managed system, or resources
- `fault_subject`: environment object or robot subsystem
- `fault_attribute`: the targetable property that changes
- `manifestation`: stuck, noisy, or degrading
- `space_domain`: isolated, contiguous, or scattered when the fault is spatial
- `time_domain`: transient, permanent, or intermittent when the fault is temporal
- `trigger`: simulation time or robot-state predicate
- `core_metrics`: details below under **Core Metrics**
- `additional_metrics`: mission-specific metrics requested by the user or needed for the scenario
- `outcome_assessment`: PASS, DEGRADED, or FAIL
- `report`: the generated Markdown file under the bind-mounted host `log/` folder with the results of running the scenario, including metric values and logged signals
- `baseline_map_assessment`: a short statement of which baseline goals and obstacles already exist in the active world and how they affect route feasibility, deadline realism, and uncertainty placement
- `fault_attribution_rule`: explicit rule for deciding whether a detection or reaction is attributable to the injected uncertainty rather than baseline hazards, nominal behavior, or unrelated monitor violations
- `control_rationale_rule`: explicit rule for how the scenario records and classifies control-command changes such as `goal_alignment`, `obstacle_avoidance`, `replan_execution`, `monitor_enforcement`, or `unknown`

## Core Metrics

Define measurable metrics for autonomy evaluation:

- **Adaptation speed**: Time in milliseconds between uncertainty injection and the first attributable rover reaction
- **Reaction attribution status**: Boolean indicating whether the credited reaction can be distinguished from nominal behavior, baseline hazards, or unrelated monitor-triggered behavior
- **Control rationale at reaction**: The logged rationale attached to the credited control response, such as `goal_alignment`, `obstacle_avoidance`, `replan_execution`, `monitor_enforcement`, or `unknown`
- **Safety preservation**: Key-value pairs with safety constraints from monitors and boolean preservation state
- **Goal viability**: Key-value pairs with mission goal and boolean indicating goal viability
- **Recovery rate**: Time in milliseconds between rover reaction to triggered uncertainty and reaction outcome
- **Detection attribution status**: Boolean indicating whether the credited detection can be distinguished from baseline hazards, unrelated obstacle signals, or other confounding runtime events
- **Minimum fault distance at detection**: Rover-to-injected-fault distance in meters when detection is credited
- **Baseline-confound status**: Boolean or short string indicating whether a baseline obstacle, monitor violation, or unrelated runtime condition could explain the credited detection or reaction
- **Additional mission-specific metrics**: Any extra metrics requested by the user or needed for the scenario, each with an explicit unit or boolean status and a short description

Fault or reaction detections must satisfy scenario-specific attribution checks such as expected sensing range, relative geometry, and consistency with the injected fault subject.

Motion reactions must not be credited from `cmd_vel` deviation alone. A credited rover reaction must be supported by a logged control rationale and by scenario state that distinguishes injected-fault response from nominal goal alignment, baseline hazards, monitor enforcement, or unrelated runtime behavior.

If attribution is ambiguous, report the relevant detection or reaction metric as `AMBIGUOUS` or `NOT ATTRIBUTABLE` instead of presenting a numeric value as though it were confidently caused by the injected uncertainty.

If there is not enough information to infer any of these fields from the mission description, BT, monitors, battery, perception, and world packages, ask the user for clarification instead of making assumptions.
