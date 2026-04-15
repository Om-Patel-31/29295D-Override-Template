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

  d(Direction::Forward, 24.0);
  t(90.0);
  sd(36.0, 18.0, 90.0);
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
- d(Direction::Forward)
- d(Direction::Forward, 20.0)
- d(Direction::Forward, 20.0, 60.0, 2000, tuning::kDrivePID)
- t(90.0)
- t(90.0, 1500, tuning::kTurnPID)
- sd(24.0, 36.0, 180.0)
- sd(24.0, 36.0, 180.0, 55.0, 3000, tuning::kSDriveTune)

Full-length:
- driveDistance(...)
- turnToHeading(...)
- sDriveTo(...)
