#ifndef QUTEE_NORMALISED_HPP_
#define QUTEE_NORMALISED_HPP_

// Qutee with normalised joints: each joint's movement range maps to [-1, 1], for controllers that work in
// normalised units (homeokinesis).
//
//   degrees = centre + n * half_width     (centre and half width of the joint's range)
//
// e.g. in the "walking" range a hip given 1 goes to 45 deg, and a knee given -1 goes to 0 deg.
// Actions are clipped to [-1, 1], so the motors stay in the range. Sensors use the same scale but are not
// clipped: a joint outside the range (pushed by hand, or at rest) reads beyond +-1.
//
// Degrees as in the rest of the firmware: 0 = the standing pose; joint 2 positive moves the leg down,
// the knee (joint 3) positive moves it forward; the same on all four legs.

#include "Qutee.hpp"

class QuteeNormalised : public Qutee
{
public:
  struct JointRange { float min_deg, max_deg; };
  // One range per kind of joint: hip (motors x1), joint 2 (x2), knee (x3)
  struct Range { const char* name; JointRange hip, joint, knee; };
  static const Range WALKING;  // hip [-45, 45], joint [-45, 45], knee [0, 90]

  typedef Eigen::Matrix<float, NB_MOTORS, 1> Joints;  // normalised values, in the order of DXL_IDs

  QuteeNormalised(const char* app_name, uint32_t ros_client_key_salt, const Range& range = WALKING);

  const Range& range() const {return _range;}
  const JointRange& joint_range(size_t motor) const;

  // One motor, between the firmware's scale (1 = 45 deg, see Qutee::Actions_t) and normalised units
  float normalise(size_t motor, float position) const;    // not clipped
  float denormalise(size_t motor, float normalised) const; // clipped to [-1, 1] first

  // The 12 joint positions, normalised (reads the motors)
  void read_normalised(Joints& sensors);
  // Send normalised targets (clipped to [-1, 1])
  void send_normalised(const Joints& actions);
  // The pose of normalised targets, in the firmware's scale (e.g. for move_slowly)
  Actions_t pose(const Joints& normalised) const;
  // Move slowly to the centre of the range (all joints at 0), where a normalised controller starts
  void go_to_range_centre();

private:
  Range _range;
};

#endif
