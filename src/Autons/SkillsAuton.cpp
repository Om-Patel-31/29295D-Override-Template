#include "framework/MotionAPI.h"

namespace framework {
namespace autons {

void skills() {
  driveToCoordinates(24.0, 0.0, 0.0);
  turnToHeading(90.0);
  driveToCoordinates(24.0, 24.0, 90.0);
  turnToHeading(180.0);
}

}  // namespace autons
}  // namespace framework
