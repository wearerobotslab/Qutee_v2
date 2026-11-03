#ifndef QUTEE_HOMEOKINESIS_HPP_
#define QUTEE_HOMEOKINESIS_HPP_

// Qutee with the homeokinesis controller (build with QUTEE_APP=homeokinesis), from the intrinsic_motivations
// submodule (components/intrinsic_motivations/homeokinesis/cpp/portable).
//
// The controller runs on the robot at CONTROL_FREQUENCY Hz in its own task: sensors are the 12 joint
// positions, motors the 12 joint targets, both normalised to the "walking" range (see QuteeNormalised).
// Start and stop it from the menu or over ROS:
//   ros2 param set /<robot name>/qutee_node running true
// Its settings are ROS 2 parameters of the same node, and can be changed while it runs.

#include "QuteeNormalised.hpp"
#include "homeokinesis.hpp"

class QuteeHomeokinesis : public QuteeNormalised
{
public:
  // Settings that can be changed over ROS while the controller runs
  // Defaults: tuned on the robot (on the floor) on 2026-10-10; the controller's own defaults are in homeokinesis.hpp
  struct HkSettings {
    float epsA = 0.5f, epsC = 0.5f;             // learning rates of the model (A) and controller (C)
    float creativity = 0.1f, harmony = 0.1f, damping = 0.00001f, causeaware = 0.01f;
    int steps4Delay = 10, steps4Averaging = 10;  // in control steps (50 Hz), 1..9
    float amplitude = 0.8f;                      // motor targets = controller output * amplitude (1 = the whole range)
    float motorSmoothing = 0.0f;                 // 0 off; s: each step the target moves (1 - s) of the way to the output
    // White noise, uniform in [-n, n], in normalised units (1 = half a joint's range: 45 deg in "walking"):
    // homeokinesis amplifies fluctuations, and these servos are too precise to provide them. Added to each
    // joint reading, and to each output before amplitude.
    float sensorNoise = 0.05f;
    float motorNoise = 0.05f;
    // Servos: time (ms) to reach each new target (0 = as fast as possible; 20 = one control period, so the
    // joints move continuously instead of jumping and stopping 50 times a second), and position P gain
    int servoProfileMs = 20;
    int servoPGain = 400;  // the XL330's own default; the poses and the checkup use 1600 (set_PID_gains)
    // Maximum output of each motor (PWM Limit, 0..885 = 0..100%): limits the load that leads to overloads.
    // An EEPROM item, so it is changed at rest with the torque off; it also applies while standing up.
    int servoPwmLimit = 700;  // about 79%; motors otherwise run at 885 (100%)
    // On an overload, reboot only that motor, let it cool down 2 s and continue (3 trips in a minute: stop)
    bool autoRecover = true;
  };

  QuteeHomeokinesis();

  bool running() const {return _running;}  // controller driving the motors
  void request_running(bool running){_run_request = running ? 1 : 0;}
  void request_reset(){_reset_request = true;}
  HkSettings settings(){std::lock_guard<std::mutex> lock(_settings_lock); return _settings;}
  void set_settings(const HkSettings& s){std::lock_guard<std::mutex> lock(_settings_lock); _settings = s; _settings_changed = true;}

protected:
  void start() override;
  std::vector<MenuItem> app_menu_items() override;
  void on_app_menu_item(size_t index) override;
  size_t ros_parameters() override {return 18;}  // see ros_setup()
  void ros_setup(rcl_node_t* node, rclc_executor_t* executor, rclc_parameter_server_t* params) override;
  bool ros_parameter_changed(const Parameter* param) override;
  void ros_spin(rclc_parameter_server_t* params) override;

private:
  typedef im::hk::Homeokinesis<float, NB_MOTORS, NB_MOTORS> Homeokinesis_t;

  static void task(void* robot);
  void loop();
  bool start_running();
  void stand_up();
  void stop_running(const char* reason);
  void apply_pwm_limit(int limit);
  void motors_initialised();

  Homeokinesis_t _hk;
  std::mutex _settings_lock;  // guards _settings, set from the ROS task
  HkSettings _settings;
  std::atomic<bool> _settings_changed{true};
  std::atomic<bool> _running{false}, _reset_request{false};
  std::atomic<int> _run_request{-1};  // -1 none, 0 stop, 1 start
  std::atomic<bool> _transition{false}; // standing up or going to rest
  bool _running_published = false;    // value of the "running" parameter (ROS task only)

  // Servo settings as last written to the motors (init_motors sets profile 0, P gain 1600, PWM limit 885)
  int _servo_profile_ms = 0, _servo_p_gain = 1600, _pwm_limit = 885;
};

#endif
