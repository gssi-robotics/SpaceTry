# spacetry_world — Package-Specific Agent Instructions

This package contains the Gazebo world configuration and models for the Mars outpost simulation environment.

## Package Overview

The `spacetry_world` package provides:
- **mars_outpost.sdf** — Main simulation world with terrain, models, and physics
- **Model definitions** — Gazebo models for terrain features, solar panels, and hazards
- **Launch configurations** — World launch files for integrated rover simulation

## Critical Files

- `worlds/mars_outpost.sdf` — World SDF definition (validates physics, lighting, models, plugins)
- `models/` — Local Gazebo models (details of available models in [src/spacetry_world/README.md](README.md))
- `CMakeLists.txt` — Package build configuration (installs SDF and models to share directory)

## Docker Interaction

All commands that validate or test the world must run in Docker:

```bash
# Validate SDF syntax and model URIs (see Verification below)
docker compose -f docker/docker-compose.yaml exec spacetry bash -lc "source /opt/ros/spaceros/setup.bash && source /etc/profile && source /ws/install/setup.bash && /ws/scripts/verify_world.sh"

# Launch the world headless for testing
docker compose -f docker/docker-compose.yaml exec spacetry bash -lc '\
  source /opt/ros/spaceros/setup.bash && source /etc/profile && source /ws/install/setup.bash && \
  timeout 10 ros2 launch spacetry_bringup spacetry_curiosity_outpost.launch.py \
    headless:=1 spawner_node:=false 2>&1 | head -50
'
```

## Impact on Test Scenarios

Modifications to `spacetry_world` affect autonomous capabilities:

- **Environmental Complexity** — Adding/removing obstacles, terrain features, or hazards tests obstacle avoidance autonomy
- **Model Availability** — Removing models breaks mission targets; ensure all objects referenced in `scenarios/mission_*/` exist in the world
- **Physics Parameters** — Changes to gravity, friction, or time scaling affect rover mobility autonomy
- **Lighting & Visibility** — Affects perception-based autonomy (sensor degradation tests rely on modifiable sensor properties)

### Baseline Map Responsibilities for Scenario Generation

When scenario drivers use this package as context, they must first understand the baseline map that already exists in `worlds/mars_outpost.sdf`.

- Treat world entities already placed in `mars_outpost.sdf` as part of the evaluated baseline environment.
- Cross-check prompt goals and mission waypoints against existing world entities before defining new goals, deadlines, or obstacle injections.
- If a prompt goal already exists in both the mission config and the world, such as `science_rock_01`, treat it as the canonical target under evaluation.
- Account for known baseline hazards, such as `block_island`, when reasoning about nominal routes, detours, and whether an injected obstacle is genuinely new.
- Validate that the deadline and timeout used by a scenario are realistic for the route from the launch point, such as `dock_pad_01`, to the evaluated goal in the existing world layout.
- Do not add or instruct scenario code to add duplicate mission targets or redundant obstacles unless the scenario explicitly studies intensified clutter around a known baseline object.

### Validation Checklist for World Changes

Before submitting world modifications:
- [ ] Are all mission targets (from scenarios/) still reachable?
- [ ] Have prompt/reference goals been checked against existing world entities and mission waypoints so scenarios do not duplicate baseline targets?
- [ ] Have baseline hazards and obstacle fields been reviewed before adding new scenario-specific obstacles?
- [ ] Are route length, start pose, and scenario deadlines still realistic for the active world layout?
- [ ] Have physics parameters been documented if changed?
- [ ] Does the world still load without errors in Gazebo?
- [ ] Do all model URIs resolve (no missing external dependencies)?
- [ ] Do nested model assets resolve for local models, including meshes, material scripts, and textures referenced from model SDF files?
- [ ] If any model is intended for runtime spawning, has it been validated both from its installed file path and with Gazebo resource-path resolution enabled?
- [ ] If runtime insertion is part of the workflow, is success confirmed by Gazebo entity listing or equivalent world-side verification rather than process exit code alone?
- [ ] Have autonomy test scenarios been re-run to verify impact?

## Verification: Validate World SDF and Model URIs

**Purpose:** Ensure the world SDF is syntactically valid and all model URIs resolve before changes are submitted.

**When to Run:**
- After modifying `worlds/mars_outpost.sdf` (any world configuration change)
- After adding/renaming models in `models/` (ensure URIs still match)
- Before creating pull requests or submitting world changes for evaluation

**Verification Steps:**

### Recommended Verification Path

A verification script is provided: `scripts/verify_world.sh`

Run from the repo root with Docker:
```bash
docker compose -f docker/docker-compose.yaml exec spacetry bash -lc "source /opt/ros/spaceros/setup.bash && source /etc/profile && source /ws/install/setup.bash && /ws/scripts/verify_world.sh"
```

Or run in a fresh container from the repo root:
```bash
docker run --rm --platform linux/amd64 \
  -v "$(pwd)":/ws -w /ws \
  spacetry:dev bash -lc '
  set -e
  source /opt/ros/spaceros/setup.bash
  source /etc/profile
  colcon build --packages-select spacetry_world
  source /ws/install/setup.bash
  /ws/scripts/verify_world.sh
'
```

This script:
1. Installs Gazebo Harmonic (if not available)
2. Fetches external models (Curiosity rover assets from space-ros/demos)
3. Validates SDF syntax
4. Checks that all model URIs are resolvable
5. Reports results (PASS/FAIL for each check)

When adding or updating models that may be spawned dynamically, also validate nested asset references inside each model's `model.sdf`, not just top-level `model://model_name` world references. This includes mesh paths, material scripts, and texture URIs.

### Fallback Manual Verification

If the automated script is unavailable, perform these steps manually in the container:

**Step 1: Set up environment**
```bash
docker compose -f docker/docker-compose.yaml exec spacetry bash -lc '\
  source /opt/ros/spaceros/setup.bash && source /etc/profile && source /ws/install/setup.bash && bash
'
```

Or enter the container directly:
```bash
docker compose -f docker/docker-compose.yaml exec spacetry bash
source /opt/ros/spaceros/setup.bash && source /etc/profile
cd /ws
```

**Step 2: Install Gazebo (if not already available)**
```bash
apt-get update && apt-get install -y gz-harmonic
```

**Step 3: Fetch external models**
```bash
set -e
DEMOS_COMMIT="acf369b"
mkdir -p /tmp/demos_src /tmp/ext_models
curl -sL "https://github.com/space-ros/demos/archive/${DEMOS_COMMIT}.tar.gz" \
  | tar xz -C /tmp/demos_src --strip-components=1
cp -a /tmp/demos_src/curiosity_rover/curiosity_gazebo/models/* /tmp/ext_models/
echo "External models fetched: $(ls /tmp/ext_models/)"
```

**Step 4: Validate SDF syntax**
```bash
export GZ_SIM_RESOURCE_PATH="/ws/src/spacetry_models/models:/tmp/ext_models"
export SDF_PATH="/ws/src/spacetry_models/models:/tmp/ext_models"
gz sdf -k /ws/src/spacetry_world/worlds/mars_outpost.sdf
echo "✓ SDF syntax valid"
```

**Step 5: Check model URIs resolve**
```bash
MODEL_DIR="/ws/src/spacetry_models/models"
EXT_DIR="/tmp/ext_models"
SDF_FILE="/ws/src/spacetry_world/worlds/mars_outpost.sdf"

MODELS=$(grep -oP 'model://\K[^</"]+' "$SDF_FILE" | sort -u)
MISSING=0

echo "Checking model URIs:"
for model in $MODELS; do
  if [ -d "$MODEL_DIR/$model" ]; then
    echo "  ✓ $model (repo)"
  elif [ -d "$EXT_DIR/$model" ]; then
    echo "  ✓ $model (external)"
  else
    echo "  ✗ MISSING: $model"
    MISSING=1
  fi
done

if [ "$MISSING" -ne 0 ]; then
  echo "ERROR: One or more model URIs could not be resolved"
  exit 1
else
  echo "✓ All model URIs resolved"
fi
```

**Step 6: Validate runtime-spawned models and nested assets**
```bash
export GZ_SIM_RESOURCE_PATH="/ws/src/spacetry_models/models:/tmp/ext_models"
export SDF_PATH="/ws/src/spacetry_models/models:/tmp/ext_models"

# Prefer relative paths inside a model package for meshes and textures that live
# under the same model directory. If model:// URIs are used inside model.sdf,
# verify they resolve with the resource path above.
grep -nE 'model://|<uri>|_map>' /ws/src/spacetry_models/models/rock_5/model.sdf

# Optional runtime insertion check for models intended to be spawned during scenarios.
timeout 10 ros2 launch spacetry_bringup spacetry_curiosity_outpost.launch.py headless:=1 > /tmp/world_launch.log 2>&1 &
LAUNCH_PID=$!
sleep 5
ros2 run ros_gz_sim create -world mars_outpost -name runtime_validation_rock -allow_renaming true \
  -file /ws/src/spacetry_models/models/rock_5/model.sdf -x 0 -y 0 -z 0
sleep 2
gz model --list | grep runtime_validation_rock
kill $LAUNCH_PID || true
```

## Known Constraints

**World Dependencies:**
- `mars_outpost.sdf` references external models from `spacetry_models/models/` (local)
- `mars_outpost.sdf` references Curiosity rover models from space-ros/demos (external, auto-fetched during build)
- Gazebo version: `harmonic` (defined in Docker image)

**SDF Validation Constraints:**
- `gz sdf -k` requires Gazebo Harmonic to be installed
- External models must be available at build time or validation will fail
- Model URIs must follow format: `model://model_name` (no spaces, lowercase preferred)
- Nested asset URIs inside model SDF files are not fully covered by top-level world URI checks; meshes, textures, and material scripts must be verified separately when models are added or changed
- Models spawned with `ros_gz_sim create -file` may resolve assets differently from models loaded directly by the world file, so runtime-spawn validation is required when scenarios depend on dynamic insertion

**Physics Simulation Constraints:**
- `use_sim_time:=true` is required for synchronized ROS2/Gazebo operation
- Some plugins (ros_gz bridges) depend on specific Gazebo clock configuration
- Changing world time scale affects all nodes depending on `/clock` topic

## Contribution Notes

Before submitting changes to `spacetry_world`:

1. **Run Verification** — Execute `scripts/verify_world.sh` to validate SDF and model URIs:
   ```bash
   docker compose -f docker/docker-compose.yaml exec spacetry bash -lc "source /opt/ros/spaceros/setup.bash && source /etc/profile && source /ws/install/setup.bash && /ws/scripts/verify_world.sh"
   ```
2. **Test in Simulation** — Launch the world in Gazebo headless mode to ensure no runtime errors
3. **Verify Runtime-Spawned Models** — If the change affects models that scenarios spawn dynamically, confirm Gazebo can insert the model at runtime and that `gz model --list` or an equivalent world-side query shows the inserted entity
4. **Check World-Specific Autonomy Impact** — Verify that mission targets remain reachable and that world changes do not silently invalidate autonomy evaluation
5. **Check Prompt-to-World Consistency** — If scenario generation depends on prompt goals or reference scenarios, verify they align with the actual world entities and mission waypoints already present
6. **Document Changes** — If adding new models, obstacles, landmarks, or physics parameters, update this file
7. **Update Mission Targets** — If modifying waypoint locations or adding new landmarks, verify scenarios still reference valid targets

### Example: Adding a New Obstacle

If adding a new rock or hazard to the world:

1. Create model in `src/spacetry_models/models/new_hazard/` with `model.config` and `model.sdf`
2. Add to `mars_outpost.sdf`: `<model name="new_hazard">...`
3. Run verification:
   ```bash
   docker compose -f docker/docker-compose.yaml exec spacetry bash -lc "source /opt/ros/spaceros/setup.bash && source /etc/profile && source /ws/install/setup.bash && /ws/scripts/verify_world.sh"
   ```
4. If the obstacle will be spawned dynamically during scenarios, validate runtime insertion and confirm its presence with `gz model --list`
5. Test impact on autonomy scenarios (may block waypoints or require replanning)
6. Update documentation if autonomy evaluation changes

### Example: Changing Physics Parameters

If modifying gravity, friction, or simulation time scale:

1. Edit appropriate `<physics>` section in `mars_outpost.sdf`
2. Run verification:
   ```bash
   docker compose -f docker/docker-compose.yaml exec spacetry bash -lc "source /opt/ros/spaceros/setup.bash && source /etc/profile && source /ws/install/setup.bash && /ws/scripts/verify_world.sh"
   ```
3. Re-run autonomy test scenarios to measure performance delta
4. Document the rationale for changes in this file
5. Ensure graceful degradation still works (rover can operate even if physics change)

---

**Related Files:**
- [Project AGENTS.md](../../AGENTS.md) — Project-wide rules and Docker requirements
- [.AGENTS.md.template](../../.AGENTS.md.template) — Template for other package-specific AGENTS.md
- [Verification Script](../../scripts/verify_world.sh) — World validation script
- [README.md](README.md) — Package description and usage
- [Scenario Driver Skill](../../skills/spacetry-autonomy-scenario-driver/SKILL.md) — Scenario-specific workflow
- [Scenario Prompt Template](../../skills/spacetry-autonomy-scenario-driver/assets/SCENARIO_PROMPT_TEMPLATE.md) — How scenarios interact with the world

**Last Updated:** April 8, 2026
