#pragma once

#include "framework/Odometry.h"
#include "framework/PID.h"
#include "framework/Tuning.h"
#include "vex.h"

namespace framework {

class ChassisController {
 public:
  ChassisController(vex::motor& leftFront, vex::motor& leftBack,
                    vex::motor& rightFront, vex::motor& rightBack,
                    OdometryTracker& odom);

  void stop();

  void driveDistance(double distanceInches, double velocityPct, int timeoutMs,
                     const tuning::PIDTune& tune);

  void turnToHeading(double targetHeadingDeg, int timeoutMs,
                     const tuning::PIDTune& tune);

  void driveToCoordinates(double targetX, double targetY, double targetHeadingDeg,
                          double velocityPct, int timeoutMs,
                          const tuning::SDriveTune& tune);

  Pose2D getPose() const;

 private:
  vex::motor& m_leftFront;
  vex::motor& m_leftBack;
  vex::motor& m_rightFront;
  vex::motor& m_rightBack;
  OdometryTracker& m_odom;

  static double clamp(double value, double minVal, double maxVal);
  void spinPercent(double leftPct, double rightPct);
};

}  // namespace framework
