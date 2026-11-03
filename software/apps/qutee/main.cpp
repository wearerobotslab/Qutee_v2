// The base robot on its own, without a controller (build with QUTEE_APP=qutee): boot, rest pose, WiFi and
// ROS (status service, torque_off_at_rest parameter), and the menu (name, checkup, IMU calibration).
// Useful to test the robot, and the starting point for a new application.
#include "Qutee.hpp"

static Qutee robot("Base robot", 0x51540000);  // "QT": its own micro-ROS session

extern "C" void app_main()
{
  robot.run(); // never returns
}
