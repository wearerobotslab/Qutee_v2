#include "QuteeNormalised.hpp"

#include <algorithm>
#include "esp_log.h"

static constexpr float DEG_PER_UNIT = 45.0f; // the firmware's scale: 1 = 45 deg

const QuteeNormalised::Range QuteeNormalised::WALKING = {
  "walking",
  {-45.0f, 45.0f},  // hip
  {-45.0f, 45.0f},  // joint 2
  {0.0f, 90.0f},    // knee
};

QuteeNormalised::QuteeNormalised(const char* app_name, uint32_t ros_client_key_salt, const Range& range):
  Qutee(app_name, ros_client_key_salt),
  _range(range)
{
}

// Motors are ordered leg by leg (11, 12, 13, 21, ...): hip, joint 2, knee
const QuteeNormalised::JointRange& QuteeNormalised::joint_range(size_t motor) const {
  switch(motor % 3){
    case 0: return _range.hip;
    case 1: return _range.joint;
    default: return _range.knee;
  }
}

float QuteeNormalised::normalise(size_t motor, float position) const {
  const JointRange& r = this->joint_range(motor);
  float centre = 0.5f * (r.min_deg + r.max_deg);
  float half_width = 0.5f * (r.max_deg - r.min_deg);
  return (position * DEG_PER_UNIT - centre) / half_width;
}

float QuteeNormalised::denormalise(size_t motor, float normalised) const {
  const JointRange& r = this->joint_range(motor);
  float centre = 0.5f * (r.min_deg + r.max_deg);
  float half_width = 0.5f * (r.max_deg - r.min_deg);
  return (centre + std::clamp(normalised, -1.0f, 1.0f) * half_width) / DEG_PER_UNIT;
}

void QuteeNormalised::read_normalised(Joints& sensors){
  State_t state;
  this->get_motor_positions(state, 6);
  for(size_t i = 0; i < NB_MOTORS; i++)
    sensors(i) = this->normalise(i, state(6 + i, 0));
}

Qutee::Actions_t QuteeNormalised::pose(const Joints& normalised) const {
  Actions_t actions;
  for(size_t i = 0; i < NB_MOTORS; i++)
    actions(i, 0) = this->denormalise(i, normalised(i));
  return actions;
}

void QuteeNormalised::send_normalised(const Joints& actions){
  this->send_actions(this->pose(actions));
}

void QuteeNormalised::go_to_range_centre(){
  ESP_LOGI("Normalised", "Moving to the centre of the %s range", _range.name);
  this->move_slowly(this->pose(Joints::Zero()), CONFIG_POSE_TRANSITION_TIME_MS);
}
