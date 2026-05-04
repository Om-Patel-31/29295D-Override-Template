#include "framework/MotionAPI.h"

namespace framework {
namespace autons {

void rightSide() {
  const Direction driveDirection = Direction::Forward;
  const double driveDistanceInches = 24.0;
  const double turnTargetHeadingDeg = 270.0;
  const double targetX = 20.0;
  const double targetY = -30.0;
  const double targetHeadingDeg = 270.0;

  driveDistance(driveDirection, driveDistanceInches);
  turnToHeading(turnTargetHeadingDeg);
  driveToCoordinates(targetX, targetY, targetHeadingDeg);
}

}  // namespace autons
}  // namespace framework
