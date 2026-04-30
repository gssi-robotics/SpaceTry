# Execution Lifecycle

Use this reference when defining how a generated scenario moves from code generation to build, validation, launch, completion, interruption, and shutdown.

## Purpose

These rules keep scenario execution:

- build-gated before validation
- launch-complete instead of leaving helper processes behind
- process-clean on nominal completion and interruption
- artifact-complete for both normal and interrupted runs

## Build Handoff

Treat the implementation-to-validation handoff as a hard build gate.

- After scenario code generation or any repo-local runtime-package edit, do not launch interrupted validation, `smoke`, `tuning`, or `full_run` yet.
- First copy every updated runtime package into `/ws/src` in the running container.
- Rebuild those updated packages successfully inside the container before any validation or execution launch.
- If that rebuild fails, remain in implementation/fix mode instead of treating the scenario as ready for validation.

This applies both to the scenario package itself and to any repo-local runtime helper package used by the scenario.

## Launch Lifecycle Ownership

When a scenario launch starts baseline bringup, the scenario driver, rosbag capture, or other helper processes together, the launch must also own scenario completion shutdown.

- The launch must emit a launch `Shutdown` event when the scenario driver process exits after finalizing artifacts.
- Do not leave bringup, rosbag, or other launch actions running indefinitely after the scenario driver exits.
- Validate this with a driver-completion path, not only with manual interruption.

Treat the absence of a driver-exit shutdown path as a launch defect.

## Node Shutdown Ownership

Scenario-driver shutdown must be process-clean as well as artifact-complete.

- `finalize()` may be reached from nominal completion, timeout, or interruption.
- Internal timers and callbacks may set termination reason and call `finalize()`.
- `rclpy.shutdown()` must have one centralized top-level owner, typically `main()` or equivalent.
- Internal callbacks, timers, or multiple exception paths must not each call `rclpy.shutdown()` independently.

An interrupted-run validation is not considered clean if the scenario driver exits with avoidable double-shutdown or repeated-context-teardown errors even when artifacts were written.

## Required Interrupted-Run Behavior

If a scenario run is stopped by `SIGINT`, `SIGTERM`, launch shutdown, timeout, or user interruption, the driver must still write:

- the scenario report Markdown file
- the metrics JSON file
- the runtime timeline or equivalent event log

Interrupted-run reports must explicitly mark the termination reason as `interrupted`, `signal`, `timeout`, or equivalent.

## Validation Expectations

Before considering the scenario launch itself valid, confirm all of the following:

- updated runtime packages were copied into `/ws/src`
- those packages were rebuilt successfully
- the launch shuts down when the driver exits
- interrupted validation still writes the required artifacts
- nominal completion and interruption are both process-clean on the scenario side

## Scope

This reference is about execution lifecycle behavior only.

- Use `Validation_and_Execution.md` for Docker commands, preflight usage, and run-wrapper usage.
- Use `Scenario_Driver_Policies.md` for hard policy enforcement.
- Use `SKILL.md` for workflow sequencing and when to load this reference.
