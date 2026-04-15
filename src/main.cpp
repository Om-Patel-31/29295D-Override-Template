/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       om31d                                                     */
/*    Created:      4/14/2026, 1:23:59 PM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include "framework/AutonManager.h"
#include "framework/MotionAPI.h"
#include "framework/RobotConfig.h"
#include "framework/Tuning.h"

using namespace vex;
using namespace framework;

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {
  configureHardware();
  initializeFramework();

  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1, 1);
  Brain.Screen.print("Calibrating inertial...");

  InertialSensor.calibrate();
  while (InertialSensor.isCalibrating()) {
    wait(1, msec);
  }

  initializeAutonProgram();
  selectAutonWithController();

  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1, 1);
  Brain.Screen.print("Auton: %s", selectedAutonName());
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
  runSelectedAuton();
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  // Basic tank drive while preserving autonomous framework architecture.
  while (1) {
    const double leftCmd = Controller1.Axis3.position(percent);
    const double rightCmd = Controller1.Axis2.position(percent);

    LeftFrontMotor.spin(fwd, leftCmd, pct);
    LeftBackMotor.spin(fwd, leftCmd, pct);
    RightFrontMotor.spin(fwd, rightCmd, pct);
    RightBackMotor.spin(fwd, rightCmd, pct);

    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  framework::Competition.autonomous(autonomous);
  framework::Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
