
## Project Structure Overview

```
spacetry/
├── AGENTS.md                        - this file with project-wide rules
├── README.md                        - project overview and usage steps
├── REF_SCENARIO.md                  - reference scenario description for autonomy evaluation
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
│       ├── scripts/                - skill-local execution wrappers and helper scripts
│       └── references/
│           ├── SCENARIO_PROMPT_QUICK_REF.md - quick reference for scenario prompt specification
│           ├── Gazebo.md                     - Gazebo world and sensor capabilities reference
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
└── scripts/                         - Utility scripts (build, testing, run, preflight, and maintained scenario wrappers)
```

## Dependency And Runtime Provenance

The SpaceTry container has two different code-provenance paths that matter during scenario experiments.

### Imported Dependencies

- `deps/spacetry.repos` pins imported repositories.
- `docker/Dockerfile` runs `vcs import /ws/src < /ws/deps/spacetry.repos` during image build.
- The imported `space_ros_demos` tree is copied into `/opt/spacetry_deps/space_ros_demos`.
- `/etc/profile.d/spacetry_deps_link.sh` recreates `/ws/src/space_ros_demos` as a runtime symlink to that cached copy.
- `COLCON_IGNORE` files are written into the cached dependency tree so only the intended subset participates in the build.

This means code under `/ws/src/space_ros_demos` may come from the image cache rather than directly from the host repository.

### Repo-Local Packages

Repo-local packages under `src/` are different:

- they are copied into the image at build time
- they are not automatically refreshed in the running container
- they must be copied into `/ws/src/<package>` with `docker cp` when the host version changes and the running container should see the update

### Why This Matters

An experiment can be stale in two different ways:

- the Docker image is old, so imported dependencies and image-copied repository files are old
- the running container is old relative to the host package copy in `src/`

Use the maintained skill-local helper scripts to make this visible:

- `skills/spacetry-autonomy-scenario-driver/scripts/scenario_preflight.sh` records the canonical skill-tree checksum by default and checks image freshness, Docker auth health, optional skill checksum or commit pinning, and host-versus-container package sync
- `skills/spacetry-autonomy-scenario-driver/scripts/run_scenario_full.sh` labels runs as `full_run`, `smoke`, or `tuning` and prevents shortened executions from being treated as the main trusted run
