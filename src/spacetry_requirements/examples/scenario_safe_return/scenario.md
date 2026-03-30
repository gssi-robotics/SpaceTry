# Scenario: Safe Return Under Low Battery

## Natural-language mission description

The rover is exploring the terrain and its battery level drops below a
critical threshold.  When this happens, the rover must abandon its current
exploration task and navigate to the safe haven waypoint.  During the return
the rover must still avoid obstacles and must not exceed a safe velocity.
The rover must eventually reach the safe haven.

**Note:** This scenario deliberately references battery signals that are
NOT YET IMPLEMENTED in SpaceTry.  It serves as a demonstration of how the
FRETish agent flags unverified signals and advises revisions.

## Environment

- Same Mars outpost terrain.
- Battery percentage signal: NOT IMPLEMENTED (planned).
- Safe haven waypoint: (-5, -5, tolerance 1.0 m).
- Obstacle detection via lidar.

## Derived minimal safety rules (plain English)

1. **Obstacle stop** (guarantee): same as rock survey scenario.
2. **No forward while obstacle** (guarantee): same as rock survey scenario.
3. **Velocity cap** (guarantee): same as rock survey scenario.
4. **Battery return** (guarantee): when battery_is_low, the rover must
   eventually reach the safe haven.
5. **Environment clearance** (assumption): same as rock survey scenario.
6. **Battery decreases** (assumption): the environment guarantees that
   battery_percentage is monotonically non-increasing unless the rover
   is at a charging location.

## Expected agent behavior

The agent should:
- Accept the requirements but flag `battery_percentage` and `battery_is_low`
  as **unverified / not implemented**.
- Warn that REQ_BATTERY_RETURN and ASM_BATTERY_DECREASE depend on signals
  that don't exist yet.
- Suggest either: (a) implement the battery model, or (b) remove/defer
  those requirements until the battery package is available.
- Still export valid artifacts for the obstacle/velocity requirements.

## Assumptions and unresolved dependencies

- `battery_percentage`: requires a future `spacetry_battery` package.
- `battery_is_low`: derived from `battery_percentage < LOW_THRESHOLD`.
- The threshold value for "low battery" is not specified — would need
  to be set by mission designers.
