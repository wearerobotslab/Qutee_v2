#ifndef JOINT_READER_HPP_
#define JOINT_READER_HPP_

// Joint reader: a minimal Qutee application (menuconfig: QUTEE settings -> Application). It turns the
// motors' torque off so the legs can be moved by hand, starts ROS at boot (no menu), and publishes the
// 12 joint positions. A simplified version of Qutee.hpp/.cpp: only the screen and the motors.

#include <array>
#include <string>

#undef HIGH // macros from the Arduino library
#undef LOW
#include "Arduino.h"
#include <Adafruit_ST7789.h>

#include "Dynamixel2Arduino.h"
#include "QuteeDxlPortHandler.hpp"

class JointReader
{
public:
  static constexpr int N = 12;
  static const int IDs[N];  // leg x 10 + joint, as on the robot's body
  typedef std::array<float, N> Angles;

  JointReader();
  void init();  // screen and motors (torque off)

  // Positions in degrees from the neutral (standing) pose: 0 deg = 2048 counts, 1 count = 0.088 deg.
  // Returns how many motors answered (a SyncRead stops at the first that doesn't; the others keep
  // their last value).
  int read_positions(Angles& degrees);

  // On: the motors hold where the joints are now. Off: the legs move freely.
  void set_torque(bool on);
  bool torque() const {return _torque;}

  void show(const Angles& degrees, int answered);  // the 12 angles on the screen
  void status(const char* msg, uint16_t color = ST77XX_WHITE);
  std::string name() const {return _name;}

private:
  void init_screen();
  void load_name();

  QuteeDxlPortHandler _port;
  Dynamixel2Arduino _dxl;
  Adafruit_ST7789 _tft;
  std::string _name;
  bool _torque = false;
  Angles _last = {};

  // SyncRead of Present Position (address 132, 4 bytes) from the 12 motors
  struct __attribute__((packed)) Position { int32_t value; };
  Position _positions[N];
  DYNAMIXEL::InfoSyncReadInst_t _sr;
  DYNAMIXEL::XELInfoSyncRead_t _sr_xels[N];
  uint8_t _sr_buf[128];
};

#endif
