# Validation And Execution

Use Docker for all build, validation, and execution steps.

## Validation

Rebuild the scenario driver related packages if scenario components have changed or new ones were added.

Apply Docker rebuild rules in this order:

- if `docker/`, `deps/`, or any non-scenario package under `src/` changed, rebuild the `spacetry:dev` image with `bash scripts/build.sh` and recreate the container before executing the scenario
- if only repo-local runtime packages such as `src/spacetry_scenario_*` changed, keep the existing image, copy each updated runtime package into `/ws/src`, and rebuild those packages inside the running container

Before running a `full_run` that should count as the main trusted result for the current scenario iteration, prefer the maintained readiness check:

```bash
scripts/scenario_preflight.sh \
  --scenario-package spacetry_scenario_{scenario_name} \
  --runtime-package spacetry_scenario_metrics \
  --run-class full_run \
  --required-skill-checksum <skill-tree-sha256> \
  --require-main-run-ready
```

This preflight checks:

- whether the current shell can actually reach the Docker daemon
- Docker registry auth health for the base image path
- whether the local `spacetry:dev` image exists
- whether the local image is newer than the latest baseline image-owned source change
- whether the scenario container is running
- whether the running container matches the current local `spacetry:dev` image
- the current canonical checksum of the skill tree by default
- whether the skill is pinned to the required checksum when exact skill-state pinning was requested
- the latest skill commit and whether the tracked skill files are dirty
- whether every declared runtime package matches the copy under `/ws/src` in the running container
- whether every declared runtime package under `/ws/install` is at least as new as the current `/ws/src` copy

During iterative tuning, when only files inside repo-local runtime packages changed, agents may use a lighter validation loop:

- copy the updated runtime package or packages into the running container
- rebuild only those runtime packages instead of the full workspace
- rerun the scenario launch or targeted validation needed to check the modified runtime-package behavior
- do not repeat earlier world verification or unrelated package validation unless those artifacts also changed

This lighter loop is allowed only when the edits are confined to repo-local runtime packages and do not change baseline autonomy packages, world files, or cross-package interfaces.

Before Docker execution, validate the scenario's ROS-consumption design:

- scenario contract/config YAML files are passed to the driver by file path only
- no plain scenario YAML file is passed directly as a ROS params file
- if a ROS params file is ever used, it must be intentionally ROS-native and structurally separate from the scenario contract/config artifacts

Before any validation search or file inspection step, keep result-reporting outputs separated from implementation inputs:

- do not inspect `log/` or `logs/` while diagnosing implementation gaps unless the implementation phase is already complete
- when checking build inputs, inspect only canonical source directories
- when checking execution outputs, inspect only current-run artifacts and do not feed them back into implementation design as source templates

If a runtime-related package is new or has changed on the host, copy it into the running container first:

```bash
docker cp $(pwd)/src/spacetry_scenario_{scenario_name} docker-spacetry-1:/ws/src/
```

Use bootstrap build mode when `/ws/install/setup.bash` does not exist yet, such as the first build in a fresh container:

```bash
docker compose -f docker/docker-compose.yaml exec spacetry bash -lc "source /opt/ros/spaceros/setup.bash && source /etc/profile && colcon build --merge-install --event-handlers console_direct+"
```

Use overlay build mode after the workspace install tree already exists:

```bash
docker compose -f docker/docker-compose.yaml exec spacetry bash -lc "source /opt/ros/spaceros/setup.bash && source /etc/profile && source /ws/install/setup.bash && colcon build --merge-install --event-handlers console_direct+"
```

If you changed `src/spacetry_world`, also run:

```bash
docker compose -f docker/docker-compose.yaml exec spacetry bash -lc "source /opt/ros/spaceros/setup.bash && source /etc/profile && source /ws/install/setup.bash && /ws/scripts/verify_world.sh"
```

## Execution Guidelines

- Use Docker for all execution. This ensures consistency and reproducibility across host machines.
- Agents must rebuild the container image with `bash scripts/build.sh` before scenario execution whenever baseline image-owned inputs changed, unless the user explicitly says to reuse an existing image.
- Before executing a scenario launch, confirm every updated runtime package has been copied into `/ws/src` and that the workspace has already been built so `/ws/install/setup.bash` exists.
- Before trusting a running container, confirm it is running from the current local `spacetry:dev` image, not a stale older image with the same tag.
- Scenario launches intended for maintained execution tooling should expose `output_root`, `run_label`, and `record_rosbag` launch arguments. Trusted `full_run` executions should use host-visible paths under `/ws/log`.
- Validate incrementally, starting with the launch and adjusting launch configuration and parameters if needed. Then move into the full scenario uninterrupted execution.
- Prefer the maintained wrapper for labeled execution:

```bash
scripts/run_scenario_full.sh \
  --launch-package spacetry_scenario_{scenario_name} \
  --launch-file scenario_{scenario_name}.launch.py \
  --scenario-package spacetry_scenario_{scenario_name} \
  --runtime-package spacetry_scenario_metrics \
  --required-skill-checksum <skill-tree-sha256> \
  --require-main-run-ready
```

- For `full_run`, the maintained wrapper now treats main-run readiness as mandatory by default. If preflight reports stale baseline image inputs, a running container from the wrong image, or out-of-sync `/ws/src` or `/ws/install` runtime package content, fix that state before launching.

- Use `--run-class smoke` or `--run-class tuning` for intentionally shortened runs. Those runs are labeled as non-main results by design and must not be reported as the primary experiment outcome.
- `full_run` executions must not use `--interrupt-after`. Any intentionally shortened execution belongs in `smoke` or `tuning`.
- A `full_run` counts as a main-result candidate only when the launch exits naturally and the metrics report an allowed natural termination reason such as `goal_reached` or `timeout`.
- During incremental launch validation, confirm that contract/config file paths arrive as ordinary string ROS parameters and that the node does not receive those YAML files via `--params-file`.
- Before claiming injected-autonomy results, confirm from runtime artifacts that the rover actually encountered the injected uncertainty according to the scenario contract's `encounter_rule`.
- Before classifying injected-autonomy failure, confirm from runtime artifacts that the run also satisfied the scenario contract's `meaningful_evaluation_rule` and that the report includes `evaluation_window_after_encounter_s`.
- Before classifying autonomy outcome, separate:
  - whether a real autonomy reaction occurred
  - what `observed_control_rationale` best describes that maneuver
  - what `reaction_scope` best describes the active baseline-plus-injected context
  - whether that reaction is attributable to injected uncertainty, baseline uncertainty, or both
- Keep encounter and fair evaluability separate:
  - if the rover never encountered the injected uncertainty, classify the injected outcome as `UNTESTED`
  - if the rover encountered the injected uncertainty but too little time remained after encounter to expect detection, reaction, or recovery evidence, classify the injected outcome as `INCONCLUSIVE` or `UNTESTED` per the contract
  - classify the injected outcome as `FAIL` only when encounter occurred and the post-encounter observation window was long enough for a fair evaluation
- If runtime artifacts show that only baseline uncertainties were exercised, report that explicitly and classify the injected-autonomy portion as `UNTESTED`.
- If a baseline obstacle or monitor clearly explains the observed maneuver, still report the maneuver classification in `observed_control_rationale`; do not force it to `unknown` merely because the injected uncertainty was not the cause.
- Store the generated report in the bind-mounted host `log/` folder with the name `spacetry_scenario_{scenario_name}_report.md`, ideally inside `log/spacetry_scenario_{scenario_name}/`, so it is accessible from the host machine after running the scenario.
- If necessary, mount the `log/` folder as a volume in Docker, for example `-v $(pwd)/log:/ws/log`, to ensure that rosbags, metrics files, runtime logs, and the final report are saved to the host machine.
- Make sure all the folders and files needed for the report are written under bind-mounted Docker volumes so they are accessible from the host machine after running the scenario.
- The created or updated scenario driver package should be copied into the running Docker container before it is built or executed.
- The scenario runtime budget must not be shorter than the baseline BT evaluation horizon. If the baseline mission or BT configuration expects a longer run to express its nominal autonomy behavior, the scenario timeout must be at least that long.

### Interrupted Runs

- If for some reason the scenario execution is interrupted, confirm that interrupted runs still writes:
  - the scenario report Markdown file
  - the metrics JSON file
  - the runtime timeline or equivalent event log
- Use an in-container interruption method for this validation:
  - launch the scenario inside the running `spacetry` container
  - identify the scenario driver PID or the scenario launch PID inside the container
  - send `SIGINT` to that in-container process
  - then verify that the report, metrics, and timeline files were written under the bind-mounted host `log/` folder
- Do not treat termination of the outer host-side wrapper command alone, such as killing `docker compose exec` from the host, as the canonical interrupted-run validation method. That mainly tests host-side process plumbing and may not deliver the intended signal to the driver.
- Temporary helper scripts or shell wrappers created only for execution orchestration are allowed for validation when they make PID capture, signal delivery, log polling, or cleanup more reliable. Keep them short-lived, execution-only, and separate from scenario implementation artifacts.
- If an interrupted run does not produce the required artifacts, treat that as a scenario-driver defect and fix it before considering the scenario complete.

## Final Reporting

When the scenario driver implementation is complete, run the associated launch file to execute the scenario and generate the report with the defined metrics and logged signals.

Use the project-wide Docker and execution instructions from `AGENTS.md`, and specify the scenario driver launch file path.

In the final answer, report the following:

- which tasks were performed
- what scenario was created or updated
- what assumptions were made
- what baseline autonomy logic was intentionally left untouched and evaluated
- which Docker validations ran
- whether baseline uncertainty, injected uncertainty, or both were actually exercised during execution
- any remaining traceability or observability gaps

The generated execution report should be placed under the bind-mounted host `log/` folder, preferably at `log/spacetry_scenario_{scenario_name}/spacetry_scenario_{scenario_name}_report.md`.
