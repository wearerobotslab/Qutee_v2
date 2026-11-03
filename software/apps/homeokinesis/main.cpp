// Qutee with the homeokinesis controller, tunable over ROS (build with QUTEE_APP=homeokinesis).
#include "QuteeHomeokinesis.hpp"

static QuteeHomeokinesis robot;

extern "C" void app_main()
{
  robot.run(); // never returns
}
