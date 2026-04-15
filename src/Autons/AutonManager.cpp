#include "framework/AutonManager.h"
#include "framework/MotionAPI.h"
#include "framework/RobotConfig.h"
#include "vex.h"

namespace framework {
namespace autons {

void safe();
void leftSide();
void rightSide();
void skills();
void test();

}  // namespace autons

namespace {

struct AutonOption {
  const char* name;
  void (*run)();
};

AutonOption g_options[] = {
    {"Safe", autons::safe},
    {"Left Side", autons::leftSide},
    {"Right Side", autons::rightSide},
    {"Skills", autons::skills},
    {"Test", autons::test},
};

  constexpr int kAutonCount = sizeof(g_options) / sizeof(g_options[0]);

int g_selectedIndex = 0;

void drawSelection() {
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1, 1);
  Brain.Screen.print("Auton: %s", g_options[g_selectedIndex].name);
  Brain.Screen.newLine();
  Brain.Screen.print("Left/Right to change");
  Brain.Screen.newLine();
  Brain.Screen.print("A to confirm");

  Controller1.Screen.clearScreen();
  Controller1.Screen.setCursor(1, 1);
  Controller1.Screen.print("Auton:");
  Controller1.Screen.setCursor(2, 1);
  Controller1.Screen.print("%s", g_options[g_selectedIndex].name);
}

}  // namespace

void initializeAutonProgram() {
  g_selectedIndex = 0;
}

void selectAutonWithController() {
  bool leftWasPressed = false;
  bool rightWasPressed = false;

  drawSelection();

  while (!Controller1.ButtonA.pressing()) {
    const bool leftPressed = Controller1.ButtonLeft.pressing();
    const bool rightPressed = Controller1.ButtonRight.pressing();

    if (leftPressed && !leftWasPressed) {
      g_selectedIndex = (g_selectedIndex + kAutonCount - 1) % kAutonCount;
      drawSelection();
    }

    if (rightPressed && !rightWasPressed) {
      g_selectedIndex = (g_selectedIndex + 1) % kAutonCount;
      drawSelection();
    }

    leftWasPressed = leftPressed;
    rightWasPressed = rightPressed;
    wait(20, vex::msec);
  }

  while (Controller1.ButtonA.pressing()) {
    wait(20, vex::msec);
  }

  Controller1.rumble(".");
  Controller1.Screen.clearScreen();
  Controller1.Screen.setCursor(1, 1);
  Controller1.Screen.print("Selected:");
  Controller1.Screen.setCursor(2, 1);
  Controller1.Screen.print("%s", g_options[g_selectedIndex].name);
}

void runSelectedAuton() {
  resetAutonomousState(0.0, 0.0, InertialSensor.heading(vex::deg));
  g_options[g_selectedIndex].run();
}

const char* selectedAutonName() {
  return g_options[g_selectedIndex].name;
}

}  // namespace framework
