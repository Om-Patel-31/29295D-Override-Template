#include "framework/MotionAPI.h"

namespace framework {
namespace autons {

void safe() {
  driveDistance(Direction::Forward, 6.0, 30.0, 1000, tuning::kDrivePID);
}

}  // namespace autons
}  // namespace framework
