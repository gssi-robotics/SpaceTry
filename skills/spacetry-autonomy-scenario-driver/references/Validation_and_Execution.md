# Validation And Execution

Use Docker for all build, validation, and execution steps.

## Validation

Rebuild the scenario driver related packages if scenario components have changed or new ones were added.

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
- Store the generated report in the bind-mounted host `log/` folder with the name `spacetry_scenario_{scenario_name}_report.md`, ideally inside `log/spacetry_scenario_{scenario_name}/`, so it is accessible from the host machine after running the scenario.
- If necessary, mount the `log/` folder as a volume in Docker, for example `-v $(pwd)/log:/ws/log`, to ensure that rosbags, metrics files, runtime logs, and the final report are saved to the host machine.
- Make sure all the folders and files needed for the report are written under bind-mounted Docker volumes so they are accessible from the host machine after running the scenario.
- The created or updated scenario driver package should be copied into the running Docker container before it is built or executed.

## Interrupted-Run Validation

- Execute at least one intentionally interrupted run for every completed scenario driver.
- Confirm that the interrupted run still writes:
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
- any remaining traceability or observability gaps

The generated execution report should be placed under the bind-mounted host `log/` folder, preferably at `log/spacetry_scenario_{scenario_name}/spacetry_scenario_{scenario_name}_report.md`.
