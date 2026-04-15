# VEX V5 Motion Framework Architecture

## Design Goals

- Beginner-first motion calls for autonomous code.
- Team-grade layered architecture for maintainability.
- Sensor-driven localization and control using external sensors only.
- Centralized tuning surface with no hidden constants.

## Layer Overview

### 1) Public Motion API Layer

Files:
- include/framework/MotionAPI.h
- src/framework/MotionAPI.cpp

Responsibilities:
- Expose shorthand commands d, t, sd.
- Expose full-length equivalents for readability.
- Keep defaults simple while allowing inline tune overrides.
- Provide fusion toggles and pose accessors.

### 2) Chassis Control Layer

Files:
- include/framework/ChassisController.h
- src/framework/ChassisController.cpp

Responsibilities:
- Execute closed-loop motion commands.
- Clamp output commands and stop motors at command end.
- Enforce timeout-safe control loops.
- Coordinate combined translation and heading control during sd.

### 3) PID Controller Layer

Files:
- include/framework/PID.h
- src/framework/PID.cpp

Responsibilities:
- Calculate PID output from error and loop dt.
- Apply anti-windup integral clamp.
- Track P/I/D term breakdown for debugging.

### 4) Odometry Tracker Layer

Files:
- include/framework/Odometry.h
- src/framework/Odometry.cpp

Responsibilities:
- Maintain x, y, heading state.
- Update from 4 rotation sensors plus inertial heading.
- Fuse GPS pose and distance displacement data as required sensors.
- Provide reset, setPosition, heading normalization utilities.

### 5) PID Tuning Config Layer

File:
- include/framework/Tuning.h

Responsibilities:
- Hold all tunables in one place only.
- Define PID gains, limits, tolerances, fusion alphas, and sd behavior constants.

### 6) Device and Competition Wiring Layer

Files:
- include/framework/RobotConfig.h
- src/framework/RobotConfig.cpp
- src/main.cpp

Responsibilities:
- Define motors and allowed sensors.
- Handle pre-auton calibration and lifecycle wiring.
- Provide demonstration autonomous routines.

## Public API Philosophy

- Calls should read like intent, not control theory.
- Beginners should start with d/t/sd defaults.
- Advanced users can override tuning inline per command.
- Full-length APIs stay available when clarity matters.

## Sensor Source Compliance

Control and localization rely on:
- inertial sensor
- 4 rotation sensors (2 left, 2 right)
- gps sensor
- distance sensor

Motor integrated encoders are not used for odometry or PID distance logic.
