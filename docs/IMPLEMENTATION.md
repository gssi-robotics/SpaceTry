
## Project Structure Overview

```
spacetry/
├── AGENTS.md                        - this file with project-wide rules
├── README.md                        - project overview and usage steps
├── deps/
│   └── spacetry.repos              - repository configuration for dependency management
├── docker/
│   ├── docker-compose.yaml         - main compose file (defines spacetry service)
│   ├── Dockerfile                  - container build config
│   ├── entrypoint.sh               - container startup script
│   └── spacetry_ws_overlay.sh      - ROS2 workspace setup script
├── skills/
│   └── spacetry-autonomy-scenario-driver/  - autonomy scenario testing framework
│       ├── SKILL.md                - skill definition and usage
│       ├── agents/                 - agent configurations
│       ├── assets/
│       │   └── SCENARIO_PROMPT_TEMPLATE.md  - template for autonomous test scenario specification
│       └── references/
│           ├── SCENARIO_PROMPT_QUICK_REF.md - quick reference for scenario prompt specification
│           ├── repo-map.md                  - repository autonomy overview
│           ├── space-fault-model.md         - fault modeling reference
│           └── Uncertainty_Taxonomy.md      - uncertainty classification taxonomy
├── src/
│   ├── spacetry_battery/            - Battery manager node
│   ├── spacetry_bringup/            - Rover launch configurations
│   ├── spacetry_bt/                 - Behavior tree runner node (C++)
│   ├── spacetry_mission/            - Mission description and configuration files
	│   ├── spacetry_models/             - Gazebo models (target rocks, obstacles, and outpost)
│   ├── spacetry_monitors/           - Safety properties monitoring node
│   ├── spacetry_perception/         - Perception nodes
│   └── spacetry_world/              - Gazebo world configurations
├── docs/
│   └── IMPLEMENTATION.md            - Project structure and implementation details
├── logs/                            - Scenario execution and test run logs
└── scripts/                         - Utility scripts (build, testing, run)
```