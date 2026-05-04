# SpaceTry BT Runner — Separate Container Setup

This setup allows you to run the behavior tree runner on a separate machine from the rover simulation. The BT runner communicates with the rover via ROS 2 DDS on the network.

## Setup

```
GSSI Workstations (ROS 2 + Gazebo):
  - Gazebo simulation with mars_outpost world
  - Curiosity rover spawned (no BT runner)
  - Controllers, battery manager, perception nodes
  - Listening on ROS_DOMAIN_ID=<id> (0, 1, or 2)

Your machine (BT Runner):
  - spacetry_bt_runner only
  - Connects to rover on Machine 1 via ROS 2 network
  - Same ROS_DOMAIN_ID as rover (0, 1, or 2)
  - Same middleware (rmw_cyclonedds_cpp)
```

## Prerequisites

1. **Both machines on the same network** with IP connectivity
2. **Rover already running** on GSSI Workstations with `enable_bt_runner:=false` (see [README.md](README.md) for rover setup)
3. **Your Machine**: Docker and the SpaceTry repository (or just the built `spacetry:bt-runner` image)
4. **Same ROS_DOMAIN_ID** on both machines (0, 1, or 2)

## Run Instructions

### Step 1: Build the BT Runner Image

First, build the minimal BT runner image:
```bash
docker build -f docker/Dockerfile.bt-runner -t spacetry:bt-runner .
```

### Step 2: Run with docker-compose (Recommended)

```bash
ROS_DOMAIN_ID=1 docker compose -f docker/docker-compose-bt-runner.yaml up
```
> Choose a specific ROS_DOMAIN_ID (0, 1, or 2) to distribute the load to the different workstations.


## Network Configuration

For the BT runner to connect to the rover on another machine:

1. **Ensure both machines can ping each other**:
   ```bash
   ping <other-machine-ip>
   ```

2. **Same ROS Domain ID**: Both must use the same ROS_DOMAIN_ID (0, 1, or 2)
   - Rover: Set via `ros2 launch` arguments or environment variable
   - BT Runner: Set via `--domain-id` argument or `ROS_DOMAIN_ID` environment variable

3. **Same Middleware**: Both use `rmw_cyclonedds_cpp` (pre-configured)

4. **Enable Discovery**: 
   - If behind a firewall, ensure UDP port 7400-7410 are open (CycloneDDS discovery)
   - Or use static peers if on isolated networks (advanced)

## Troubleshooting

### "No nodes found on other machines"
- Check that both machines can reach each other: `ping <other-machine-ip>`
- Verify same ROS_DOMAIN_ID on both machines (use 0, 1, or 2)
- Verify middleware: `ros2 topic list -m` should show connection info

### BT runner connects but rover doesn't move
- Check battery level on rover (needs >15% to move)
- Verify `/cmd_vel` topic is being published: `ros2 topic echo /cmd_vel`
- Check monitor constraints and safety conditions on rover

### Slow/Intermittent Communication
- Check network latency: `ping -c 5 <other-machine-ip>`
- Reduce wifi interference (use 5GHz if available)
- Consider using ethernet for stable cross-machine communication
- Ensure no firewall blocking UDP 7400-7410 (CycloneDDS discovery)

## Advanced: Custom Tree File

To use a different behavior tree, edit `docker-compose-bt-runner.yaml` and change the `tree_file` parameter in the command:
```yaml
command:
  - bash
  - -lc
  - 'source /opt/ros/jazzy/setup.bash && ... && ros2 run spacetry_bt spacetry_bt_runner --ros-args -p tree_file:=/ws/src/spacetry_bt/trees/escape_profile_bt.xml ...'
```

## Logs

Logs from the BT runner are saved to:
- Your machine local path: `./log/` (mounted as `/ws/log` in container)

View logs in real-time:
```bash
docker logs -f docker-spacetry-bt-runner-1
```

## Clean Up

Stop the BT runner:
```bash
# For docker-compose
docker compose -f docker/docker-compose-bt-runner.yaml down

# For standalone containers
docker stop <container-id>
```

For running the whole ROS 2 + Gazebo workspace, see [README.md](README.md#step-by-step-run-spacetry--with-the-curiosity-rover)
