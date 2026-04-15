#include "framework/RobotConfig.h"

namespace framework {

vex::brain Brain;
vex::competition Competition;
vex::controller Controller1(vex::primary);

vex::motor LeftFrontMotor(vex::PORT1, vex::ratio18_1, false);
vex::motor LeftBackMotor(vex::PORT2, vex::ratio18_1, false);
vex::motor RightFrontMotor(vex::PORT9, vex::ratio18_1, true);
vex::motor RightBackMotor(vex::PORT10, vex::ratio18_1, true);

vex::inertial InertialSensor(vex::PORT11);
vex::rotation LeftRotationA(vex::PORT3, false);
vex::rotation LeftRotationB(vex::PORT4, false);
vex::rotation RightRotationA(vex::PORT5, true);
vex::rotation RightRotationB(vex::PORT6, true);
vex::gps GpsSensor(vex::PORT12);
vex::distance FrontDistanceSensor(vex::PORT13);

void configureHardware() {
  LeftFrontMotor.setStopping(vex::brake);
  LeftBackMotor.setStopping(vex::brake);
  RightFrontMotor.setStopping(vex::brake);
  RightBackMotor.setStopping(vex::brake);
}

}  // namespace framework
