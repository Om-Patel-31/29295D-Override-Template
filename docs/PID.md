# PID Guide

## Plain-Language Equation

The controller output is the sum of three terms:

output = P + I + D

where:
- P = kP * error
- I = kI * accumulated error over time
- D = kD * rate of change of error

Think of this as:
- P pushes toward the target now.
- I fixes persistent bias that never quite reaches target.
- D damps the motion to reduce overshoot.

## Anti-Windup Strategy

Integral windup happens when error remains large for too long and the I term grows too much.
This framework clamps the accumulated integral state to a configurable range.
That keeps recovery stable after long errors or saturation.

## Tuning Workflow

1. Start with kI = 0 and kD = 0.
2. Increase kP until response is quick but not unstable.
3. Add a small kD to reduce overshoot.
4. Add very small kI only if steady-state error remains.
5. Set tolerance to practical accuracy, not perfection.
6. Set integral clamp high enough to help, low enough to stay safe.

## Inline Override Usage

Defaults come from include/framework/Tuning.h.
You can override per command to test quickly.

Example for drive distance:

~~~cpp
const Direction driveDirection = Direction::Forward;
const double distanceInches = 30.0;
const double velocityPct = 55.0;
const int timeoutMs = 2000;
const tuning::PIDTune driveTune{0.92, 0.0008, 0.18, 0.5, 10.0};

d(driveDirection, distanceInches, velocityPct, timeoutMs, driveTune);
~~~

Example for turn:

~~~cpp
const double targetHeadingDeg = 180.0;
const int turnTimeoutMs = 1800;
const tuning::PIDTune turnTune{1.05, 0.0, 0.24, 1.0, 35.0};

t(targetHeadingDeg, turnTimeoutMs, turnTune);
~~~

## Debugging P I D Terms

The PID class stores lastP, lastI, lastD.
These values help identify whether oscillation is from high P, drift from low I, or sluggish response from low D.
