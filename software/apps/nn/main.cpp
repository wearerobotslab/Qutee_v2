// Qutee with a neural network policy whose weights come over ROS (build with QUTEE_APP=nn).
#include "QuteeNN.hpp"

static QuteeNN robot;

extern "C" void app_main()
{
  robot.run(); // never returns
}
