# Odometry and Fusion Guide

## Coordinate Frame

This framework uses a field-style global frame:
- x increases in the robot forward-facing global east direction
- y increases in the robot forward-facing global north direction
- heading is degrees in [0, 360)

Heading wraps safely using normalizeHeading.

## Update Pipeline

Each odometry update performs:

1. Read left and right rotation sensors (1 per side).
2. Convert rotation degrees to linear inches.
3. Compute delta-left and delta-right from previous readings.
4. Compute forward delta as average of side deltas.
5. Read inertial heading and treat it as authoritative heading.
6. Project forward delta into global x/y using heading.
7. Blend GPS x/y/heading using alpha.
8. Blend distance-based forward delta when valid.

## Why Inertial Heading Is Primary

Wheel-only heading integration drifts under slip.
Using inertial heading each cycle gives more stable orientation.
Wheel deltas are still used for forward displacement.

## GPS Fusion Behavior

When enabled:
- x and y are blended with gps x/y.
- heading is blended with gps heading.
- alpha controls trust in GPS.

Low alpha means gentle long-term correction.
High alpha means stronger correction but more jump sensitivity.

## Distance Fusion Behavior

When enabled:
- Forward displacement estimate is blended with change in front distance reading.
- Readings outside configured valid range are ignored.
- Invalid readings automatically degrade back to wheel-only displacement.

This allows safer operation if the distance target is absent or too far.

## Common Failure Cases and Fixes

1. Drift after straight runs:
- Verify rotation sensor direction signs and mounting.
- Re-check wheel diameter and track constants.
- Reduce GPS alpha if GPS noise causes jitter.

2. Position jumps:
- Reduce GPS alpha.
- Confirm GPS has clear field view and stable installation.

3. Bad forward estimate with distance fusion:
- Confirm sensor faces a consistent object plane.
- Tighten valid distance min/max range.
- Disable distance fusion for open-field segments.

4. Turn angle mismatch:
- Ensure inertial calibrates fully in pre-auton.
- Keep robot still during calibration.
