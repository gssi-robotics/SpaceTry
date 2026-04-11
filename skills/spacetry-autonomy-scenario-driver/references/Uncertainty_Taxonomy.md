# Uncertainty Taxonomy

Below we present uncertainty taxonomy from `Mahdavi-Hezavehi, Sara, Paris Avgeriou, and Danny Weyns. "A classification framework of uncertainty in architecture-based self-adaptive systems with multiple quality requirements." Managing trade-offs in adaptable software architectures. Morgan Kaufmann, 2017. 45-77.` avaialable in: https://people.cs.kuleuven.be/~danny.weyns/papers/2015MASA.pdf. Some of the dimensions names were adjusted according to their definition in the paper for better understandibility.
For the `Location` dimension of uncertainty, the types of sources of uncertainty are also listed.

Uncertainty Dimensions:
- Location:
    - Environment
        - Sources:
            - Execution Context:
                - Waypoints goals
                - Terrain
                - Hazards
            - Human-in-the-Loop
            - Third-party component
            
    - Model
        - Sources:
            - Abstraction
            - Incompleteness
            - Model Drift
            - Different Representation or Sources of Information
            - Complex Models
    - Adapation Functions:
        - Sources:
            - Variability Space of Adaptation
            - Sensing
            - Effects Not Completely Deterministic
            - Machine Learning
            - Decentralization
            - Dynamicity of Adaptation Mechanisms
            - Inaccurate Fault Localization and Identification
    - Goals:
        - Sources:
            - Future Goals Changes
            - Future New Goals
            - Goal Specification
            - Outdated Goals
    - Managed System:
        - Sources:
            - System Complexity and Changes
        - SpaceTry Managed Systems:
            - Robot subsystems
            - Sensors
            - Wheels

    - Resources:
        - Sources:
            - New Resources
            - Changing Resources
        - SpaceTry Robot Subsystems Resources:
            - Battery energy level (soc)
- Nature:
    - Epistemic
    - Variability
- Level:
    - Statistical Uncertainty
    - Scenario Uncertainty (SpaceTry)
- Emerging Time
    - Runtime
    - Design Time

## SpaceTry Operational Fault Concepts Used by the Scenario Driver

The scenario driver prompt and contract use a focused subset of the SpaceTry fault model to operationalize uncertainty in simulation. These concepts complement the taxonomy above and are the ones currently used in `SKILL.md`, `SCENARIO_PROMPT_TEMPLATE.md`, and `SCENARIO_PROMPT_QUICK_REF.md`.

- Fault subject:
    - Environment object
        - Goals
        - Waypoints
        - Hazards
            - Harsh terrain tile
            - Rock obstacle
            - Hole tile
    - Robot subsystem
        - Sensor
        - Battery
        - Wheel
        - Engine
- Fault attribute:
    - A targetable property of a fault subject that can be changed by injection
    - Examples:
        - Sensor reading
        - Battery energy level
        - Wheel rotational speed
        - Wheel wear-out
        - Engine actuator command
        - Hazard-induced stuck probability
- Manifestation:
    - Stuck:
        - The attribute is forced to a fixed value
    - Noisy:
        - The attribute is perturbed with random error
    - Degrading:
        - The attribute error increases or decreases over space or time
- Time domain:
    - Transient:
        - One short-duration occurrence over time
    - Permanent:
        - Persists across consecutive time slots
    - Intermittent:
        - Repeats across separated time intervals
- Space domain:
    - Isolated:
        - One localized spatial occurrence
    - Contiguous:
        - Multiple connected affected areas
    - Scattered:
        - Multiple separated affected areas
- Trigger or constraint:
    - Simulation time
    - Robot-state predicate or robot attributes
        - For example, speed, distance to an object, or mission-progress condition

## Mapping Between Taxonomy and Operational Concepts

- `Location: Environment` commonly maps to environment fault subjects such as goals, waypoints, and hazards.
- `Location: Managed System` commonly maps to robot-subsystem fault subjects such as sensors, wheels, engine, and battery-related state.
- `Location: Resources` commonly maps to resource attributes such as battery energy level.
- `Emerging Time` aligns with whether uncertainty is introduced at runtime or design time.
- `Manifestation`, `Time domain`, `Space domain`, `Fault attribute`, and `Trigger or constraint` refine how a selected uncertainty source is injected in a scenario.
