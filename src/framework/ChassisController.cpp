#include "framework/ChassisController.h"

#include <cmath>

#include "framework/RobotConfig.h"

namespace framework {

namespace {
constexpr double kPi = 3.14159265358979323846;
}

ChassisController::ChassisController(vex::motor& leftFront, vex::motor& leftBack,
                                     vex::motor& rightFront,
                                     vex::motor& rightBack,
                                     OdometryTracker& odom)
    : m_leftFront(leftFront),
      m_leftBack(leftBack),
      m_rightFront(rightFront),
      m_rightBack(rightBack),
      m_odom(odom) {}

void ChassisController::stop() {
  m_leftFront.stop();
  m_leftBack.stop();
  m_rightFront.stop();
  m_rightBack.stop();
}

void ChassisController::driveDistance(double distanceInches, double velocityPct,
                                      int timeoutMs,
                                      const tuning::PIDTune& tune) {
  PIDController pid(tune);
  pid.rst();

  const Pose2D start = m_odom.getPose();
  const double startHeadingRad = start.headingDeg * kPi / 180.0;
  const double startCenterToBackX =
      tuning::kPidReferenceBackOffsetInches * std::cos(startHeadingRad);
  const double startCenterToBackY =
      tuning::kPidReferenceBackOffsetInches * std::sin(startHeadingRad);
  const double startRefX = start.x - startCenterToBackX;
  const double startRefY = start.y - startCenterToBackY;

  const double targetDistanceMagnitude = std::fabs(distanceInches);
  vex::timer timer;

  while (timer.time(vex::msec) < timeoutMs) {
    m_odom.upd();
    const Pose2D now = m_odom.getPose();

    const double sensorDistanceIn = FrontDistanceSensor.objectDistance(vex::inches);
    const bool sensorValid = tuning::kUseDistanceSensorDrivePid &&
                             sensorDistanceIn >= tuning::kDistanceMinValidInches &&
                             sensorDistanceIn <= tuning::kDistanceMaxValidInches;

    double error = 0.0;
    if (sensorValid) {
      const double measuredBackDistance =
          sensorDistanceIn + static_cast<double>(tuning::kDistanceSensorBackOffsetInches);
      // Positive error means the robot is farther than target and should drive forward.
      error = measuredBackDistance - targetDistanceMagnitude;
    } else {
      const double nowHeadingRad = now.headingDeg * kPi / 180.0;
      const double nowCenterToBackX =
          tuning::kPidReferenceBackOffsetInches * std::cos(nowHeadingRad);
      const double nowCenterToBackY =
          tuning::kPidReferenceBackOffsetInches * std::sin(nowHeadingRad);
      const double nowRefX = now.x - nowCenterToBackX;
      const double nowRefY = now.y - nowCenterToBackY;

      const double dx = nowRefX - startRefX;
      const double dy = nowRefY - startRefY;
      const double projectedTravel =
          dx * std::cos(startHeadingRad) + dy * std::sin(startHeadingRad);
      error = distanceInches - projectedTravel;
    }

    const double rawCommand = pid.calc(error, tuning::kControlLoopDtSec);
    const double cappedByVelocity = clamp(rawCommand, -velocityPct, velocityPct);
    const double command =
      clamp(cappedByVelocity, -tuning::kMaxDriveCommandPct,
          tuning::kMaxDriveCommandPct);

    spinPercent(command, command);

    if (pid.ok(error)) {
      break;
    }

    vex::wait(static_cast<int>(tuning::kControlLoopDtSec * 1000.0), vex::msec);
  }

  stop();
}

void ChassisController::turnToHeading(double targetHeadingDeg, int timeoutMs,
                                      const tuning::PIDTune& tune) {
  PIDController pid(tune);
  pid.rst();

  vex::timer timer;

  while (timer.time(vex::msec) < timeoutMs) {
    m_odom.upd();
    const Pose2D now = m_odom.getPose();

    const double error = OdometryTracker::herr(targetHeadingDeg, now.headingDeg);
    const double rawTurn = pid.calc(error, tuning::kControlLoopDtSec);
    const double turn = clamp(rawTurn, -tuning::kMaxTurnCommandPct,
                  tuning::kMaxTurnCommandPct);

    spinPercent(turn, -turn);

    if (pid.ok(error)) {
      break;
    }

    vex::wait(static_cast<int>(tuning::kControlLoopDtSec * 1000.0), vex::msec);
  }

  stop();
}

void ChassisController::driveToCoordinates(double targetX, double targetY,
                                           double targetHeadingDeg,
                                           double velocityPct,
                                           int timeoutMs,
                                           const tuning::SDriveTune& tune) {
  PIDController distancePid(tune.distance);
  PIDController headingPid(tune.heading);
  distancePid.rst();
  headingPid.rst();

  vex::timer timer;

  while (timer.time(vex::msec) < timeoutMs) {
    m_odom.upd();
    const Pose2D now = m_odom.getPose();

    const double dx = targetX - now.x;
    const double dy = targetY - now.y;
    const double distanceError = std::sqrt(dx * dx + dy * dy);

    const double pathHeadingDeg = OdometryTracker::norm(
      std::atan2(dy, dx) * 180.0 / kPi);

    // Blend heading objective: far away prioritize path direction, near target prioritize final heading.
    const double blend = clamp(
        distanceError / tuning::kSDriveHeadingBlendDistanceInches, 0.0, 1.0);
    const double headingTarget =
      OdometryTracker::norm(
            blend * pathHeadingDeg + (1.0 - blend) * targetHeadingDeg);

    const double headingError =
      OdometryTracker::herr(headingTarget, now.headingDeg);

    double forward = distancePid.calc(distanceError, tuning::kControlLoopDtSec);
    double turn = headingPid.calc(headingError, tuning::kControlLoopDtSec);

        forward = clamp(forward, -velocityPct, velocityPct);
        forward = clamp(forward, -tuning::kMaxSDriveForwardPct,
            tuning::kMaxSDriveForwardPct);
        turn = clamp(turn, -tuning::kMaxSDriveTurnPct, tuning::kMaxSDriveTurnPct);

    // Swing behavior: when heading error is large, reduce inside wheel to arc cleanly.
    double left = forward + turn;
    double right = forward - turn;
    if (std::fabs(headingError) > tuning::kSDriveSwingThresholdDeg) {
      if (headingError > 0.0) {
        left *= tuning::kSDriveSwingInsideScale;
      } else {
        right *= tuning::kSDriveSwingInsideScale;
      }
    }

    spinPercent(left, right);

    const bool atDistance = distancePid.ok(distanceError);
    const bool atHeading = std::fabs(
      OdometryTracker::herr(targetHeadingDeg, now.headingDeg)) <=
        tune.heading.tolerance;
    if (atDistance && atHeading) {
      break;
    }

    vex::wait(static_cast<int>(tuning::kControlLoopDtSec * 1000.0), vex::msec);
  }

  stop();
}

Pose2D ChassisController::getPose() const { return m_odom.getPose(); }

double ChassisController::clamp(double value, double minVal, double maxVal) {
  if (value < minVal) {
    return minVal;
  }
  if (value > maxVal) {
    return maxVal;
  }
  return value;
}

void ChassisController::spinPercent(double leftPct, double rightPct) {
  m_leftFront.spin(vex::fwd, leftPct, vex::pct);
  m_leftBack.spin(vex::fwd, leftPct, vex::pct);
  m_rightFront.spin(vex::fwd, rightPct, vex::pct);
  m_rightBack.spin(vex::fwd, rightPct, vex::pct);
}

}  // namespace framework
