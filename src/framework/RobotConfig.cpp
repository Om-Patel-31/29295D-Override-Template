#include "framework/RobotConfig.h"

namespace framework {

vex::brain Brain;
vex::competition Competition;
vex::controller Controller1(vex::primary);

vex::motor LeftFrontMotor(vex::PORT5, vex::ratio6_1, false);
vex::motor LeftMiddleMotor(vex::PORT3, vex::ratio6_1, false);
vex::motor LeftBackMotor(vex::PORT4, vex::ratio6_1, false);
vex::motor RightFrontMotor(vex::PORT10, vex::ratio6_1, true);
vex::motor RightMiddleMotor(vex::PORT7, vex::ratio6_1, true);
vex::motor RightBackMotor(vex::PORT8, vex::ratio6_1, true);

vex::inertial InertialSensor(vex::PORT11);
vex::rotation LeftRotationA(vex::PORT3, false);
vex::rotation RightRotationA(vex::PORT5, true);
vex::gps GpsSensor(vex::PORT12);
vex::distance FrontDistanceSensor(vex::PORT13);

void configureHardware() {
  LeftFrontMotor.setStopping(vex::brake);
  LeftMiddleMotor.setStopping(vex::brake);
  LeftBackMotor.setStopping(vex::brake);
  RightFrontMotor.setStopping(vex::brake);
  RightMiddleMotor.setStopping(vex::brake);
  RightBackMotor.setStopping(vex::brake);
}

}  // namespace framework
