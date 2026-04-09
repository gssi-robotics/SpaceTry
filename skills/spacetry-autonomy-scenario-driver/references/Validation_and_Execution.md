# Validation And Execution

Use Docker for all build, validation, and execution steps.

## Validation

Rebuild the scenario driver related packages if scenario components have changed or new ones were added.

Before Docker execution, validate the scenario's ROS-consumption design:

- scenario contract/config YAML files are passed to the driver by file path only
- no plain scenario YAML file is passed directly as a ROS params file
- if a ROS params file is ever used, it must be intentionally ROS-native and structurally separate from the scenario contract/config artifacts

Before any validation search or file inspection step, keep result-reporting outputs separated from implementation inputs:

- do not inspect `log/` or `logs/` while diagnosing implementation gaps unless the implementation phase is already complete
- when checking build inputs, inspect only canonical source directories
- when checking execution outputs, inspect only current-run artifacts and do not feed them back into implementation design as source templates

If the scenario related package is new or has changed on the host, copy it into the running container first:

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
- Agents must rebuild the container image with `bash scripts/build.sh` before scenario execution unless the user explicitly says to reuse an existing image.
- Before executing a scenario launch, confirm the scenario package has been copied into `/ws/src` and that the workspace has already been built so `/ws/install/setup.bash` exists.
- Validate incrementally, starting with the launch and adjusting launch configuration and parameters if needed. Then move into the full scenario uninterrupted execution.
- During incremental launch validation, confirm that contract/config file paths arrive as ordinary string ROS parameters and that the node does not receive those YAML files via `--params-file`.
- Before claiming injected-autonomy results, confirm from runtime artifacts that the rover actually encountered the injected uncertainty according to the scenario contract's `encounter_rule`.
- If runtime artifacts show that only baseline uncertainties were exercised, report that explicitly and classify the injected-autonomy portion as `UNTESTED`.
- Store the generated report in the bind-mounted host `log/` folder with the name `spacetry_scenario_{scenario_name}_report.md`, ideally inside `log/spacetry_scenario_{scenario_name}/`, so it is accessible from the host machine after running the scenario.
- If necessary, mount the `log/` folder as a volume in Docker, for example `-v $(pwd)/log:/ws/log`, to ensure that rosbags, metrics files, runtime logs, and the final report are saved to the host machine.
- Make sure all the folders and files needed for the report are written under bind-mounted Docker volumes so they are accessible from the host machine after running the scenario.
- The created or updated scenario driver package should be copied into the running Docker container before it is built or executed.

### Interrupted Runs

- If for some reason the scenario execution is interrupted, confirm that interrupted runs still writes:
  - the scenario report Markdown file
  - the metrics JSON file
  - the runtime timeline or equivalent event log
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
