#include "framework/MotionAPI.h"

namespace framework {
namespace autons {

void rightSide() {
  driveDistance(Direction::Forward, 24.0);
  turnToHeading(270.0);
  driveToCoordinates(20.0, -30.0, 270.0);
}

}  // namespace autons
}  // namespace framework
