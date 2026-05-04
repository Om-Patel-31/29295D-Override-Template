# Quickstart

## Setup Checklist

1. Open include/framework/RobotConfig.h and src/framework/RobotConfig.cpp.
2. Set motor ports, gear ratios, and reversals to your robot.
3. Set inertial, 4 rotation, gps, and distance sensor ports.
4. Build and deploy once to confirm device configuration.

## Sensor Port Checklist

Required sensors and expected roles:
- inertial sensor: absolute heading source
- left rotation A and B: left drivetrain displacement
- right rotation A and B: right drivetrain displacement
- gps sensor: required x/y/heading drift correction
- distance sensor: required forward displacement refinement

## First Autonomous Example

~~~cpp
void autonomous() {
  resetForAutonomous(0.0, 0.0, InertialSensor.heading(vex::deg));
  enableGpsFusion(true);
  enableDistanceFusion(true);

  const Direction driveDirection = Direction::Forward;
  const double driveDistanceInches = 24.0;
  const double turnTargetHeadingDeg = 90.0;
  const double targetX = 36.0;
  const double targetY = 18.0;
  const double targetHeadingDeg = 90.0;

  d(driveDirection, driveDistanceInches);
  t(turnTargetHeadingDeg);
  sd(targetX, targetY, targetHeadingDeg);
}
~~~

## Tune Order Checklist

1. Confirm odometry signs and heading direction first.
2. Tune drive PID.
3. Tune turn PID.
4. Tune sd distance PID.
5. Tune sd heading PID.
6. Enable GPS fusion and set gps alpha.
7. Enable distance fusion and set distance alpha.
8. Fine-tune swing threshold and inside scale for sd.

## API Summary

Shorthand:
- d(direction, distanceInches)
- d(direction, distanceInches, velocityPct, timeoutMs, pidTune)
- t(targetHeadingDeg)
- t(targetHeadingDeg, timeoutMs, pidTune)
- sd(targetX, targetY, targetHeadingDeg)
- sd(targetX, targetY, targetHeadingDeg, velocityPct, timeoutMs, sDriveTune)

Full-length:
- driveDistance(direction, distanceInches, velocityPct, timeoutMs, pidTune)
- turnToHeading(targetHeadingDeg, timeoutMs, pidTune)
- driveToCoordinates(targetX, targetY, targetHeadingDeg, velocityPct, timeoutMs, sDriveTune)
