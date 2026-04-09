# spacetry_bringup — Package-Specific Agent Instructions

This package owns rover bringup launch integration for the Mars outpost simulation, including the baseline launch composition, BT runner startup, and scenario-facing launch arguments.

## Scope

Apply these rules whenever you:
- modify files under `src/spacetry_bringup/`
- include `spacetry_bringup` launches from a scenario package
- depend on `spacetry_bringup` launch arguments or ROS node parameter wiring during scenario execution

These rules supplement the project-wide `AGENTS.md` and are the canonical place for bringup-specific launch and parameter guidance.

## Launch Integration Rules

- Treat launch-time parameter wiring as part of scenario integration rather than assuming every bringup default is safe in every runtime environment.
- Do not redeclare `use_sim_time` inside a Python scenario node if it is already injected by launch or declared via node options. Read it if needed, but avoid redeclaration because it can raise `ParameterAlreadyDeclaredException` at runtime.
- Verify QoS compatibility for every existing SpaceTry topic that a ROS 2 node depends on for triggers, metrics, derived status signals, or integration behavior. Do not assume default ROS 2 subscription QoS will match existing publishers in this stack.
- In case of doubt, use an explicitly compatible subscriber QoS profile rather than the default reliable subscription when attaching a node to existing SpaceTry topics.
- `/mobile_base_controller/odom` is a concrete example of this rule: its publisher QoS may be best-effort in this stack, so nodes that subscribe to it for position tracking, goal checks, or injection timing should use an explicitly matched QoS profile.
- If baseline bringup starts the BT runner, verify that the `tree_file` launch argument resolves to a non-empty absolute path in the execution environment.
- If there is any doubt about `tree_file` resolution, pass the path explicitly from the scenario launch only to ensure the evaluated baseline BT is the one that actually loads at runtime.
- The BT file used for launch must remain the autonomy behavior under evaluation. Do not swap in a scenario-specific BT or an edited BT variant unless the user explicitly approves a bug fix to the baseline autonomy logic.
- Record in scenario notes whether the evaluated baseline BT path came entirely from baseline bringup or had to be explicitly re-passed by the scenario launch for runtime robustness. This is a launch-integration safeguard, not a change to the autonomy behavior being evaluated.

## Scenario-Facing Bringup Checks

- When including baseline bringup from a scenario launch, inspect bringup-side defaults before relying on launch overrides for mission-specific values such as battery SOC, odom topics, spawn targets, waypoint files, and scenario-specific thresholds.
- When a node depends on existing SpaceTry topics, inspect both the topic names and their effective QoS settings before finalizing subscriptions. If the node depends on odometry, obstacle, battery, monitor, or similar streams for injection timing, control adaptation, or report metrics, confirm the subscriber QoS matches the publisher QoS used by the stack.
- Prefer explicit scenario launch arguments over hidden assumptions when a scenario depends on bringup behavior that is sensitive to path resolution or parameter injection order.
- Keep the baseline bringup composition unchanged unless the user explicitly approves a bringup bug fix.

## Validation

- After modifying `src/spacetry_bringup/` files, validate by rebuilding in Docker and relaunching the rover or scenario that depends on those changes.
- When a scenario depends on bringup launch behavior, verify the effective launch arguments in the execution environment rather than assuming host-side path substitutions will behave the same way inside the container.
