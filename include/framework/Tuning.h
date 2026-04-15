#pragma once

namespace framework {
namespace tuning {

struct PIDTune {
  double kP;
  double kI;
  double kD;
  double tolerance;
  double integralClamp;
};

struct SDriveTune {
  PIDTune distance;
  PIDTune heading;
};

// Drivetrain geometry and motion loop constants.
inline constexpr double kTrackWidthInches = 12.0;
inline constexpr double kWheelDiameterInches = 3.25;
inline constexpr double kControlLoopDtSec = 0.02;

// Core drive distance PID defaults.
inline constexpr PIDTune kDrivePID{
    2.0,  // kP
    0.5,   // kI
    0.01,  // kD
    0.01,  // tolerance (in)
    10.0   // integral clamp
};

// Core heading turn PID defaults.
inline constexpr PIDTune kTurnPID{
    0.5,   // kP
    0.01,  // kI
    0.05,  // kD
    0.01,   // tolerance (deg)
    30.0   // integral clamp
};

// Simultaneous drive-to-point + heading tune defaults.
inline constexpr PIDTune kSDriveDistancePID{
    0.75,
    0.01,
    0.10,
    0.01,
    12.0};

inline constexpr PIDTune kSDriveHeadingPID{
    1.0,
    0.01,
    0.25,
    0.01,
    35.0};

inline constexpr SDriveTune kSDriveTune{
    kSDriveDistancePID,
    kSDriveHeadingPID};

// Safety and behavior caps.
inline constexpr double kDefaultDriveDistanceInches = 24.0;
inline constexpr double kDefaultDriveVelocityPct = 60.0;
inline constexpr int kDefaultDriveTimeoutMs = 2500;
inline constexpr int kDefaultTurnTimeoutMs = 2000;
inline constexpr int kDefaultSDriveTimeoutMs = 3500;

inline constexpr double kMaxDriveCommandPct = 80.0;
inline constexpr double kMaxTurnCommandPct = 70.0;
inline constexpr double kMaxSDriveForwardPct = 75.0;
inline constexpr double kMaxSDriveTurnPct = 60.0;

// Fusion constants (0.0 = no trust, 1.0 = full trust).
inline constexpr double kGpsFusionAlpha = 0.15;

// Distance-sensor drive PID behavior.
// Set true to use front distance sensor for driveDistance() error when valid.
inline constexpr bool kUseDistanceSensorDrivePid = true;

// Desired driveDistance() target is interpreted from the robot back reference:
// backDistance = frontDistance + kDistanceSensorBackOffsetInches.
inline constexpr int kDistanceSensorBackOffsetInches = 0;

// If distance sensor is invalid, driveDistance() falls back to odometry travel.
// This offset changes the odometry reference from center to back for PID measurement.
inline constexpr double kPidReferenceBackOffsetInches = 0.0;

// S-Drive heading blend behavior.
inline constexpr double kSDriveSwingThresholdDeg = 35.0;
inline constexpr double kSDriveSwingInsideScale = 0.55;
inline constexpr double kSDriveHeadingBlendDistanceInches = 14.0;

// Distance sensor validity window.
inline constexpr double kDistanceMinValidInches = 1.0;
inline constexpr double kDistanceMaxValidInches = 120.0;

}  // namespace tuning
}  // namespace framework
