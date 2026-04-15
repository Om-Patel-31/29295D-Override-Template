#include "framework/MotionAPI.h"

namespace framework {
namespace autons {

void leftSide() {
  driveDistance(Direction::Forward, 24.0);
  turnToHeading(90.0);
  driveToCoordinates(20.0, 30.0, 90.0);
}

}  // namespace autons
}  // namespace framework
