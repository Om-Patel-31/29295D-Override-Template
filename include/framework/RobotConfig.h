#pragma once

#include "vex.h"

namespace framework {

extern vex::brain Brain;
extern vex::competition Competition;
extern vex::controller Controller1;

extern vex::motor LeftFrontMotor;
extern vex::motor LeftBackMotor;
extern vex::motor RightFrontMotor;
extern vex::motor RightBackMotor;

extern vex::inertial InertialSensor;
extern vex::rotation LeftRotationA;
extern vex::rotation RightRotationA;
extern vex::gps GpsSensor;
extern vex::distance FrontDistanceSensor;

void configureHardware();

}  // namespace framework
