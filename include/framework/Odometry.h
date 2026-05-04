#pragma once

#include "framework/Tuning.h"
#include "vex.h"

namespace framework {

struct Pose2D {
  double x;
  double y;
  double headingDeg;
};

class OdometryTracker {
 public:
  OdometryTracker(vex::rotation& left, vex::rotation& right, vex::inertial& inertial,
                  vex::gps& gpsSensor);

  void rst(double x = 0.0, double y = 0.0, double headingDeg = 0.0);
  void set(double x, double y, double headingDeg);

  void upd();
  Pose2D getPose() const;

  static double norm(double headingDeg);
  static double herr(double targetDeg, double currentDeg);

 private:
  vex::rotation& m_left;
  vex::rotation& m_right;
  vex::inertial& m_inertial;
  vex::gps& m_gps;

  double m_x;
  double m_y;
  double m_headingDeg;

  double m_prevLeftIn;
  double m_prevRightIn;

  static double deg2in(double deg);
};

}  // namespace framework
