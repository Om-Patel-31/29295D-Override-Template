# 29295D Override Template

A beginner-friendly VEX V5 autonomous template with a layered motion framework, odometry, PID control, and competition-ready autonomous program selection.

This template is designed so you can start with simple motion calls (`d`, `t`, `sd`) while still keeping a clean architecture that scales as your codebase grows.

## What This Template Does

- Provides a motion API for common autonomous actions:
  - `d(...)` for driving a distance
  - `t(...)` for turning to heading
  - `sd(...)` for driving to an (x, y, heading) pose
- Uses a sensor-driven odometry pipeline:
  - inertial heading
   - 2 rotation sensors for drivetrain displacement
  - GPS fusion for pose drift correction
  - distance sensor fusion for forward displacement refinement
- Includes centralized tuning in one place (`include/framework/Tuning.h`)
- Includes autonomous routine management with controller-based selection before match start
- Ships with sample auton slots (`Left Side`, `Right Side`, `Skills`, `Test`)

## Template Architecture

The framework is organized into clear layers:

1. Public Motion API (`MotionAPI`)  
   High-level commands you call in auton code.
2. Chassis Control (`ChassisController`)  
   Closed-loop control execution and timeouts.
3. PID (`PID`)  
   Generic PID controller logic with anti-windup.
4. Odometry (`Odometry`)  
   Pose tracking and sensor fusion.
5. Tuning (`Tuning`)  
   All gains, limits, tolerances, and fusion constants.
6. Robot Wiring (`RobotConfig` + `main.cpp`)  
   Devices, lifecycle setup, and competition callbacks.

## Required Sensors and Roles

This template expects and uses all of the following:

- 1 inertial sensor (absolute heading)
- 2 rotation sensors (1 left, 1 right drivetrain tracking)
- 1 GPS sensor (pose fusion)
- 1 distance sensor (forward displacement fusion)

Note: Motor integrated encoders are not used for odometry or PID distance logic in this framework.

## Quick Start

1. Configure ports and reversals in:
   - `include/framework/RobotConfig.h`
   - `src/framework/RobotConfig.cpp`
2. Build and deploy once to confirm hardware setup.
3. In `pre_auton()`:
   - hardware is configured
   - inertial is calibrated
   - auton list is initialized
   - auton is selected with controller buttons
4. Write or edit routines in `src/Autons/`.
5. Run selected auton through `runSelectedAuton()` during competition autonomous.

## Example Autonomous Code

```cpp
void autonomous() {
  resetForAutonomous(0.0, 0.0, InertialSensor.heading(vex::deg));
  enableGpsFusion(true);
  enableDistanceFusion(true);

  d(Direction::Forward, 24.0);
  t(90.0);
  sd(36.0, 18.0, 90.0);
}
```

## Project Layout

- `src/main.cpp`: competition lifecycle (`pre_auton`, `autonomous`, `usercontrol`)
- `src/Autons/`: autonomous routines and auton manager
- `src/framework/`: motion, odometry, PID, and robot config implementation
- `include/framework/`: framework headers and tuning surface
- `docs/`: architecture notes, quickstart, odometry, and PID references

## Documentation

- `docs/Quickstart.md`
- `docs/Architecture.md`
- `docs/Odometry.md`
- `docs/PID.md`
- `docs/FrameworkReference.html`

## Who This Is For

- Teams that want easy-to-read autonomous code now and deeper control later
- Teams that want one consistent place for tuning
- Beginners who need a structured motion stack without starting from scratch
