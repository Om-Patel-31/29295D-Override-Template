#include "framework/MotionAPI.h"

namespace framework {

namespace {
OdometryTracker* g_odom = nullptr;
ChassisController* g_chassis = nullptr;

static double directionSign(Direction direction) {
  return (direction == Direction::Forward) ? 1.0 : -1.0;
}

}  // namespace

void initializeFramework() {
  static OdometryTracker odom(LeftRotationA, RightRotationA, InertialSensor,
                              GpsSensor);
  static ChassisController chassis(LeftFrontMotor, LeftBackMotor, RightFrontMotor,
                                   RightBackMotor, odom);

  g_odom = &odom;
  g_chassis = &chassis;
}

void resetAutonomousState(double x, double y, double headingDeg) {
  LeftRotationA.resetPosition();
  RightRotationA.resetPosition();

  if (g_odom != nullptr) {
    g_odom->rst(x, y, headingDeg);
  }
}

Pose2D getPose() {
  if (g_odom == nullptr) {
    return {0.0, 0.0, 0.0};
  }
  g_odom->upd();
  return g_odom->getPose();
}

void driveDistance(Direction direction, double distanceInches, double velocityPct,
                   int timeoutMs, const tuning::PIDTune& pidTune) {
  if (g_chassis == nullptr) {
    return;
  }

  const double signedDistance = directionSign(direction) * distanceInches;
  g_chassis->driveDistance(signedDistance, velocityPct, timeoutMs, pidTune);
}

void turnToHeading(double targetHeadingDeg, int timeoutMs,
                   const tuning::PIDTune& pidTune) {
  if (g_chassis == nullptr) {
    return;
  }

  g_chassis->turnToHeading(targetHeadingDeg, timeoutMs, pidTune);
}

void driveToCoordinates(double targetX, double targetY, double targetHeadingDeg,
                        double velocityPct, int timeoutMs,
                        const tuning::SDriveTune& sDriveTune) {
  if (g_chassis == nullptr) {
    return;
  }

  g_chassis->driveToCoordinates(targetX, targetY, targetHeadingDeg, velocityPct,
                                timeoutMs, sDriveTune);
}

}  // namespace framework
