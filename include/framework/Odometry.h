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
  OdometryTracker(vex::rotation& leftA, vex::rotation& leftB, vex::rotation& rightA,
                  vex::rotation& rightB, vex::inertial& inertial,
                  vex::gps& gpsSensor);

  void rst(double x = 0.0, double y = 0.0, double headingDeg = 0.0);
  void set(double x, double y, double headingDeg);

  void upd();
  Pose2D getPose() const;

  static double norm(double headingDeg);
  static double herr(double targetDeg, double currentDeg);

 private:
  vex::rotation& m_leftA;
  vex::rotation& m_leftB;
  vex::rotation& m_rightA;
  vex::rotation& m_rightB;
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
