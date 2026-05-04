#include "framework/Odometry.h"

#include <cmath>

namespace framework {

namespace {
constexpr double kPi = 3.14159265358979323846;
}

OdometryTracker::OdometryTracker(vex::rotation& left, vex::rotation& right,
                                 vex::inertial& inertial, vex::gps& gpsSensor)
    : m_left(left),
      m_right(right),
      m_inertial(inertial),
      m_gps(gpsSensor),
      m_x(0.0),
      m_y(0.0),
      m_headingDeg(0.0),
      m_prevLeftIn(0.0),
      m_prevRightIn(0.0) {}

void OdometryTracker::rst(double x, double y, double headingDeg) {
  m_left.resetPosition();
  m_right.resetPosition();

  m_x = x;
  m_y = y;
  m_headingDeg = norm(headingDeg);

  m_prevLeftIn = 0.0;
  m_prevRightIn = 0.0;
}

void OdometryTracker::set(double x, double y, double headingDeg) {
  m_x = x;
  m_y = y;
  m_headingDeg = norm(headingDeg);
}

void OdometryTracker::upd() {
  // Step 1: Convert the left and right rotation sensors to linear travel.
  // We only use external rotation sensors so drive motor integrated encoders never affect odometry.
  const double leftIn = deg2in(m_left.position(vex::deg));
  const double rightIn = deg2in(m_right.position(vex::deg));

  // Step 2: Compute incremental chassis motion since last update.
  const double deltaLeft = leftIn - m_prevLeftIn;
  const double deltaRight = rightIn - m_prevRightIn;
  m_prevLeftIn = leftIn;
  m_prevRightIn = rightIn;

  double deltaForward = (deltaLeft + deltaRight) * 0.5;

  // Step 3: Use inertial heading as the authoritative orientation.
  // This avoids integrating heading drift from wheel slip alone.
  m_headingDeg = norm(m_inertial.heading(vex::deg));
  const double headingRad = m_headingDeg * kPi / 180.0;

  // Step 4: Project local forward delta into global field X/Y.
  m_x += deltaForward * std::cos(headingRad);
  m_y += deltaForward * std::sin(headingRad);

  // GPS fusion is always part of the pose estimate.
  // GPS data is blended with alpha to gently correct drift rather than abruptly jump.
    const double gpsX = m_gps.xPosition(vex::inches);
    const double gpsY = m_gps.yPosition(vex::inches);
    const double gpsHeading = norm(m_gps.heading(vex::deg));

    m_x = (1.0 - tuning::kGpsFusionAlpha) * m_x +
      tuning::kGpsFusionAlpha * gpsX;
    m_y = (1.0 - tuning::kGpsFusionAlpha) * m_y +
      tuning::kGpsFusionAlpha * gpsY;
    m_headingDeg = norm((1.0 - tuning::kGpsFusionAlpha) * m_headingDeg +
            tuning::kGpsFusionAlpha * gpsHeading);
}

Pose2D OdometryTracker::getPose() const { return {m_x, m_y, m_headingDeg}; }

double OdometryTracker::norm(double headingDeg) {
  double wrapped = std::fmod(headingDeg, 360.0);
  if (wrapped < 0.0) {
    wrapped += 360.0;
  }
  return wrapped;
}

double OdometryTracker::herr(double targetDeg, double currentDeg) {
  const double raw = norm(targetDeg) - norm(currentDeg);
  if (raw > 180.0) {
    return raw - 360.0;
  }
  if (raw < -180.0) {
    return raw + 360.0;
  }
  return raw;
}

double OdometryTracker::deg2in(double deg) {
  return (deg / 360.0) * (tuning::kWheelDiameterInches * kPi);
}

}  // namespace framework
