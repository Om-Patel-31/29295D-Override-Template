# VEX V5 Motion Framework - API Reference

**Status:** Stable

This reference defines the motion API contract, core control architecture, autonomous selection flow, and purpose of each project file. The inertial sensor is used for heading, motor encoders are used for distance tracking, and the framework is designed to be simple and beginner-friendly.

---

## Quick Start

### Basic Pre-Autonomous Setup

```cpp
void pre_auton() {
  configureHardware();
  initializeFramework();
  InertialSensor.calibrate();
  while (InertialSensor.isCalibrating()) {
    wait(50, vex::msec);
  }
}
```

### Simple Autonomous Example

```cpp
void autonomous() {
  resetAutonomousState(0.0, 0.0, InertialSensor.heading(vex::deg));
  
  driveDistance(Direction::Forward, 24.0);
  turnToHeading(90.0);
  driveDistance(Direction::Forward, 18.0);
}
```

**Common patterns:**
- Drive forward/backward a fixed distance
- Turn to face a specific heading
- Repeat to create complex autonomous routines

---

## Public API Reference

| Function | Purpose | Use Case | Example |
|----------|---------|----------|---------|
| `configureHardware()` | Apply motor stopping policies and initialize hardware. | Call once during `pre_auton()` for predictable drivetrain behavior. | `configureHardware();` |
| `initializeFramework()` | Create and bind all motion control modules to hardware. | Enable motion commands before any autonomous or driver control. | `initializeFramework();` |
| `resetAutonomousState(x, y, heading)` | Reset odometry and sensors to a known field pose. | Start autonomous from a predictable position with correct heading. | `resetAutonomousState(0.0, 0.0, 0.0);` |
| `getPose()` | Return the current robot position and heading. | Get telemetry for debug displays or conditional logic. | `Pose2D pose = getPose();` |
| `driveDistance(direction, inches, [speed], [timeout], [pidTune])` | Drive forward or backward a fixed distance with PID control. | Move off start tile, backup from wall, score objects. | `driveDistance(Direction::Forward, 24.0);` |
| `turnToHeading(targetDeg, [timeout], [pidTune])` | Rotate robot to face an absolute field heading. | Align for next action, prepare for straight drive. | `turnToHeading(90.0);` |
| `Direction::Forward` / `Direction::Reverse` | Select drive direction for distance commands. | Specify direction while keeping distance positive. | `driveDistance(Direction::Reverse, 12.0);` |
| `initializeAutonProgram()` | Reset autonomous selection to default state. | Prepare for controller-based auton choice at startup. | `initializeAutonProgram();` |
| `selectAutonWithController()` | Let driver pick autonomous routine via controller buttons. | Pit-side match setup for quick auton selection. | `selectAutonWithController();` |
| `runSelectedAuton()` | Execute the autonomous routine driver selected. | Competition callback in `autonomous()` function. | `runSelectedAuton();` |
| `selectedAutonName()` | Get the name of the currently selected autonomous. | Display selected auton on Brain screen for confirmation. | `Brain.Screen.print("%s", selectedAutonName());` |

---

## Understanding PID Control

### What is PID?

PID (Proportional-Integral-Derivative) control is a feedback system that adjusts motor power based on how far away you are from your target.

**The formula:**
```
output = P + I + D
  P (Proportional) = kP × error
  I (Integral)     = kI × sum(error over time), clamped to prevent windup
  D (Derivative)   = kD × change in error / dt
```

### Tuning the Gains

| Gain | What It Does | Too Low | Too High |
|------|-------------|---------|----------|
| **kP** (Proportional) | Makes robot respond to error | Slow response, reaches target slowly | Fast but shaky, overshoots, oscillates |
| **kI** (Integral) | Corrects persistent errors | Never quite reaches target | Overshoot and oscillation |
| **kD** (Derivative) | Dampens response, prevents overshoot | Oscillates and shakes | Very sluggish and slow |

### Default Starting Values

```cpp
// Distance control (inches)
const double DRIVE_kP = 2.0;
const double DRIVE_kI = 0.1;
const double DRIVE_kD = 0.05;

// Heading control (degrees)
const double TURN_kP = 1.5;
const double TURN_kI = 0.05;
const double TURN_kD = 0.08;
```

---

## Understanding Odometry

### What is Odometry?

Odometry tracks your robot's position and heading by reading motor encoders and the inertial sensor. This allows you to know where the robot is on the field without external sensors.

### How It Works

1. **Motors measure distance**: Left and right motor encoders track how far each side has traveled
2. **Inertial measures heading**: The inertial sensor tells us what direction robot is facing
3. **Position is calculated**: Using simple geometry, we estimate x/y position on field

### Motor Encoder Math

```
distance (inches) = rotations × wheel_diameter × π

Example:
  10 rotations × 3.25 inch diameter × 3.14159 = ~102 inches traveled
```

### Odometry Functions

```cpp
// Update robot position (call this every loop)
void update();

// Reset position to known value
void resetPose(double x, double y, double heading);

// Read current position
double getX();        // Field X (inches)
double getY();        // Field Y (inches)
double getHeading();  // Robot heading (0-360 degrees)
```

### Debugging Odometry

**Print position to Brain screen:**
```cpp
Pose2D pos = getPose();
Brain.Screen.print("X: %.1f Y: %.1f H: %.1f", pos.x, pos.y, pos.heading);
```

**Check encoder values:**
- Left motor should read similar rotations to right motor when driving straight
- If one side drifts, check wheel alignment and motor cable connections

---

## Internal Control Layers

### For Advanced Tuning

The framework uses layered control for flexibility:

1. **Public API** (`driveDistance`, `turnToHeading`) — Use these
2. **ChassisController** — Internal motion controller
3. **PID Controller** — Low-level feedback loop
4. **Odometry** — Position tracking
5. **Hardware** — Motor and sensor access

### Tuning the Internal Layers

| Layer | File | Tunable | How to Adjust |
|-------|------|---------|---------------|
| PID gains | `include/framework/Tuning.h` | `DRIVE_kP`, `DRIVE_kI`, `DRIVE_kD` | Modify constants, recompile, test |
| Motion timeouts | `include/framework/Tuning.h` | `DEFAULT_TIMEOUT_MS` | Increase if routines stop early |
| Odometry reset | `src/Autons/*.cpp` | Initial pose | Call `resetAutonomousState(x, y, h)` |
| Motor stop behavior | `src/RobotConfig.cpp` | Stop mode | Tune `motor::brakeType` |

---

## File Purpose Reference

### Core Framework

| File | Purpose |
|------|---------|
| `src/main.cpp` | Competition entry point, calibration, autonomous & driver control |
| `include/framework/MotionAPI.h` | Public motion function declarations |
| `src/framework/MotionAPI.cpp` | Public API implementation |
| `include/framework/MotionControl.h` | Drive and turn function interfaces |
| `src/framework/MotionControl.cpp` | Motion control implementation with PID loops |
| `include/framework/PID.h` | Simple PID controller class |
| `src/framework/PID.cpp` | PID math implementation |
| `include/framework/Odometry.h` | Odometry position tracking interface |
| `src/framework/Odometry.cpp` | Encoder-based position tracking |
| `include/framework/RobotConfig.h` | Hardware declarations |
| `src/framework/RobotConfig.cpp` | Motor and sensor port definitions |
| `include/framework/Tuning.h` | All PID gains and timing constants |

### Autonomous Routines

| File | Purpose |
|------|---------|
| `src/Autons/LeftAuton.cpp` | Left-side autonomous example |
| `src/Autons/RightAuton.cpp` | Right-side autonomous example |
| `src/Autons/SkillsAuton.cpp` | Skills autonomous example |
| `src/Autons/TestAuton.cpp` | Simple test routine |

### Documentation

| File | Purpose |
|------|---------|
| `docs/FrameworkReference.md` | This file — API reference |
| `docs/Quickstart.md` | Getting started guide |
| `docs/PID.md` | Detailed PID tuning guide |
| `docs/Odometry.md` | Odometry math explanation |
| `docs/Architecture.md` | Framework design philosophy |

### Build System

| File | Purpose |
|------|---------|
| `makefile` | Build configuration |
| `vex/mkenv.mk` | VEX toolchain settings |
| `vex/mkrules.mk` | VEX build rules |

---

## Common Tasks

### Write a Simple Autonomous

```cpp
void runLeftAuton() {
  // Start from known position
  resetAutonomousState(0.0, 0.0, 0.0);
  
  // Move off wall
  driveDistance(Direction::Forward, 24.0);
  
  // Turn to face goal
  turnToHeading(90.0);
  
  // Drive to goal
  driveDistance(Direction::Forward, 36.0);
}
```

### Display Robot Position During Autonomous

```cpp
void runDebugAuton() {
  resetAutonomousState(0.0, 0.0, 0.0);
  
  // Move and check position
  driveDistance(Direction::Forward, 24.0);
  
  Pose2D pos = getPose();
  Brain.Screen.print("Final X: %.1f, Y: %.1f, H: %.1f", 
                     pos.x, pos.y, pos.heading);
}
```

### Tune PID Gains

1. Edit constants in `include/framework/Tuning.h`
2. Start with default values provided
3. **Increase kP** if robot is too slow
4. **Decrease kP** if robot overshoots and shakes
5. **Increase kD** if robot oscillates
6. Recompile and test

### Debug Motor Encoder Drift

```cpp
// Check if left/right rotations match
void checkEncoders() {
  double leftRotations = LeftFrontMotor.rotation(vex::rev);
  double rightRotations = RightFrontMotor.rotation(vex::rev);
  
  Brain.Screen.print("L: %.2f R: %.2f", leftRotations, rightRotations);
  
  // They should be similar. If very different, check:
  // - Wheel sizes match
  // - Motor gearing matches
  // - Wheels aren't slipping
}
```

---

## Troubleshooting

| Problem | Likely Cause | Solution |
|---------|-------------|----------|
| Robot won't move | Motors stalled or encoder disconnected | Check motor connections, run motor test |
| Odometry drifts | Wheel slip or encoder error | Check wheel alignment, verify wheel diameter constant |
| PID oscillates | kP too high or kD too low | Decrease kP, increase kD |
| Robot overshoots target | PID gains too aggressive | Decrease kP, increase kD |
| Inertial shows wrong heading | Calibration incomplete | Call `inertialSensor.calibrate()` before autonomous |
| Drive commands timeout early | Timeout too short or friction too high | Increase `DEFAULT_TIMEOUT_MS`, check motor current |

---

## Next Steps

1. **Read** `docs/Quickstart.md` for setup checklist
2. **Edit** `include/framework/Tuning.h` with your robot's measurements
3. **Write** your first autonomous in `src/Autons/LeftAuton.cpp`
4. **Test** and tune PID gains if needed
5. **Reference** this page when implementing new features

---

For detailed PID explanation and math, see [docs/PID.md](PID.md)

For odometry coordinate system explanation, see [docs/Odometry.md](Odometry.md)

For architecture philosophy, see [docs/Architecture.md](Architecture.md)
