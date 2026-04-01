# Uncertainty Taxonomy

Below we present uncertainty taxonomy from `Mahdavi-Hezavehi, Sara, Paris Avgeriou, and Danny Weyns. "A classification framework of uncertainty in architecture-based self-adaptive systems with multiple quality requirements." Managing trade-offs in adaptable software architectures. Morgan Kaufmann, 2017. 45-77.` avaialable in: https://people.cs.kuleuven.be/~danny.weyns/papers/2015MASA.pdf. Some of the dimensions names were adjusted according to their definition in the paper for better understandibility.
For the `Location` dimension of uncertainty, the sources of uncertainty are also listed.

Uncertainty Dimensions:
- Location:
    - Environment
        - Sources:
            - Execution Context
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
    - Resources:
        - Sources:
            - New Resources
            - Changing Resources
- Nature:
    - Epistemic
    - Variability
- Level:
    - Statistical Uncertainty
    - Scenario Uncertainty
- Emerging Time
    - Runtime
    - Design Time