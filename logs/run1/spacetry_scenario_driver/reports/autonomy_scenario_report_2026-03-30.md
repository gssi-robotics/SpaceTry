# Autonomy Scenario Run Report

Date: 2026-03-30

## Run Setup

- Launch command:
  `docker compose -f docker/docker-compose.yaml exec spacetry bash -lc 'source /opt/ros/spaceros/setup.bash && source /etc/profile && source /ws/install/setup.bash && timeout 130 ros2 launch spacetry_scenario_driver autonomy_scenario.launch.py headless:=1 report_path:=/tmp/spacetry_autonomy_report.json'`
- Scenario package: `spacetry_scenario_driver`
- Scenario config: `spacetry_scenario_driver/config/autonomy_scenario.yaml`

## What Started Successfully

- Gazebo headless world launch
- Curiosity rover spawn request and successful entity creation
- ROS-Gazebo bridges for `/clock`, `/scan`, and odometry
- Battery manager
- Obstacle direction node
- Copilot monitor
- BT runner
- Scenario driver node
- Controller activation chain for the rover controllers

## Observed Runtime Behavior

- The BT runner entered `MissionSequence`, completed `SetGoal`, and entered `NavigateWithAvoidance`.
- The battery manager initialized with an initial state of charge of about 23%.
- In the longer live run, the safety monitor repeatedly fired:
  `MR_009_RETURN_TO_OUTPOST_ON_LOW_BATTERY`
- No successful completion of the mission or return-to-outpost behavior was observed during the run window.

## Safety Outcome

- `MR_009_RETURN_TO_OUTPOST_ON_LOW_BATTERY`: violated during runtime
- `MR_011_MAINTAIN_SPEED_WHEN_FULL_BATTERY`: no direct violation observed in the captured logs

## Assessment

- Overall status: Failed
- Reason:
  The rover stack entered navigation, but the safety monitor repeatedly detected a low-battery return-to-outpost violation, and the run did not recover to a safe autonomous outcome within the scenario time budget.

## Driver Report Status

- Expected automated artifact:
  `/tmp/spacetry_autonomy_report.json`
- Actual result:
  not generated during this run

## Follow-up Recommendation

- Investigate why the scenario driver does not finalize and emit its JSON report even after the configured runtime.
- Investigate why the rover remains in a state that continuously violates `MR_009`, starting from an initial battery near the threshold.
