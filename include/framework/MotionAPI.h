#pragma once

#include "framework/ChassisController.h"
#include "framework/RobotConfig.h"
#include "framework/Tuning.h"

namespace framework {

enum class Direction {
  Forward,
  Reverse
};

void initializeFramework();
void resetAutonomousState(double x = 0.0, double y = 0.0, double headingDeg = 0.0);

Pose2D getPose();

void driveDistance(Direction direction, double distanceInches,
                   double velocityPct = tuning::kDefaultDriveVelocityPct,
                   int timeoutMs = tuning::kDefaultDriveTimeoutMs,
                   const tuning::PIDTune& pidTune = tuning::kDrivePID);

void turnToHeading(double targetHeadingDeg,
                   int timeoutMs = tuning::kDefaultTurnTimeoutMs,
                   const tuning::PIDTune& pidTune = tuning::kTurnPID);

void driveToCoordinates(double targetX, double targetY, double targetHeadingDeg,
                        double velocityPct = tuning::kDefaultDriveVelocityPct,
                        int timeoutMs = tuning::kDefaultSDriveTimeoutMs,
                        const tuning::SDriveTune& sDriveTune = tuning::kSDriveTune);

}  // namespace framework
