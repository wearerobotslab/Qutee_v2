#include "QuteeHomeokinesis.hpp"

#include <string.h>
#include <thread>
#include "esp_log.h"
#include "esp_random.h"
#include "esp_timer.h"
#include <rcl_interfaces/msg/parameter_type.h>

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ESP_LOGE("UROS","Failed status on line %d: %d. Aborting.",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}

QuteeHomeokinesis::QuteeHomeokinesis():
  QuteeNormalised("Homeokinesis", 0x484B0000, QuteeNormalised::WALKING)  // "HK": its own micro-ROS session
{
}

// Stand up, then move to the centre of the range: the controller's output 0 (knees at 45 deg in "walking"),
// so it starts without a jump
void QuteeHomeokinesis::stand_up(){
  this->go_to_standing_pose();
  this->go_to_range_centre();
}

// ----- Controller task ----- //

// The controller's matrix work needs more stack than the main task has, so it runs in its own task,
// from boot. It waits until it is asked to run.
void QuteeHomeokinesis::start(){
  xTaskCreate(task, "homeokinesis", CONFIG_ROBOT_CTRL_APP_STACK, this, CONFIG_ROBOT_CTRL_APP_TASK_PRIO, NULL);
}

void QuteeHomeokinesis::task(void* robot){
  static_cast<QuteeHomeokinesis*>(robot)->loop();
  vTaskDelete(NULL);
}

// After init_motors(): the motors are back to profile 0, P gain 1600 and PWM limit 885
void QuteeHomeokinesis::motors_initialised(){
  _servo_profile_ms = 0;
  _servo_p_gain = 1600;
  _pwm_limit = 885;
}

// PWM Limit is an EEPROM item: only at rest, with the torque off (the torso is on the ground)
void QuteeHomeokinesis::apply_pwm_limit(int limit){
  if(limit == _pwm_limit)
    return;
  this->set_torque(false);
  this->set_pwm_limit(limit);
  _pwm_limit = limit;
}

// Take the motors and stand up. Returns false if another activity (e.g. the checkup) has the motors.
bool QuteeHomeokinesis::start_running(){
  if(!this->acquire_motors("homeokinesis")){
    std::string reason = std::string("homeokinesis refused: ") + (this->motors_owner() ? this->motors_owner() : "?")
                       + " is running; ";
    this->set_motor_report(reason);
    this->tft_status("Homeokinesis refused: motors busy", ST77XX_YELLOW);
    return false;
  }
  this->tft_status("Homeokinesis: standing up", ST77XX_YELLOW);
  this->set_motor_report("");
  if(this->reboot_faulted_motors(false) > 0){
    this->init_motors();
    this->motors_initialised();
  }
  this->apply_pwm_limit(this->settings().servoPwmLimit);
  this->stand_up();
  // The controller's servo settings (the poses use the defaults)
  HkSettings s = this->settings();
  _servo_profile_ms = s.servoProfileMs;
  _servo_p_gain = s.servoPGain;
  this->set_servo_motion(_servo_profile_ms, _servo_p_gain);
  _running = true;
  this->tft_status("Homeokinesis running", ST77XX_GREEN);
  ESP_LOGI("HK", "Homeokinesis running, %s range", this->range().name);
  return true;
}

// Back to rest with the servos' default settings, and free the motors. The PWM limit stays (an EEPROM
// write each time would wear the motors' EEPROM); init_motors() sets it back to 885 at boot.
void QuteeHomeokinesis::stop_running(const char* reason){
  this->go_to_rest_pose();
  if(_servo_profile_ms != 0 || _servo_p_gain != 1600){
    this->set_servo_motion(0, 1600);
    _servo_profile_ms = 0;
    _servo_p_gain = 1600;
  }
  _running = false;
  this->release_motors();
  ESP_LOGI("HK", "Homeokinesis stopped (%s)", reason);
}

void QuteeHomeokinesis::loop(){
  Joints sensors;
  Homeokinesis_t::SensorVec x;
  // Smoothed motor targets, normalised (0 = the centre of the range)
  Homeokinesis_t::MotorVec target = Homeokinesis_t::MotorVec::Zero();
  float amplitude = 0, smoothing = 0, sensor_noise = 0, motor_noise = 0;
  bool auto_recover = true;
  // Uniform white noise in [-n, n], from the hardware random number generator
  auto noise = [](float n) { return n == 0.0f ? 0.0f : (esp_random() * (2.0f / 4294967295.0f) - 1.0f) * n; };
  int64_t trips[NB_MOTORS][3] = {}; // times of each motor's last 3 overloads, for auto-recovery
  size_t fault_check = 0; // motor whose hardware error status is read this step (one per step)
  int64_t step_us_sum = 0, step_us_max = 0;
  int steps_in_log = 0;
  int64_t next = esp_timer_get_time();

  while(true){
    int request = _run_request.exchange(-1);
    if(request != -1 && (request == 1) != _running){
      _transition = true;
      if(request == 1){
        if(this->start_running()){
          target.setZero();
          step_us_sum = step_us_max = 0;
          steps_in_log = 0;
          this->reset_incomplete_reads();
        }
      } else {
        this->stop_running("requested");
        this->tft_status("Homeokinesis stopped");
      }
      _transition = false;
      next = esp_timer_get_time();
    }
    if(_reset_request.exchange(false)){
      _hk.reset();
      ESP_LOGI("HK", "Controller reset");
    }
    if(_settings_changed.exchange(false)){
      HkSettings s = this->settings();
      auto& p = _hk.params();
      p.epsA = s.epsA; p.epsC = s.epsC;
      p.creativity = s.creativity; p.harmony = s.harmony; p.damping = s.damping; p.causeaware = s.causeaware;
      p.steps4Delay = s.steps4Delay; p.steps4Averaging = s.steps4Averaging;
      amplitude = s.amplitude;
      smoothing = s.motorSmoothing;
      sensor_noise = s.sensorNoise;
      motor_noise = s.motorNoise;
      auto_recover = s.autoRecover;
      // The servo settings are written while the controller has the motors (otherwise when it starts)
      if(_running && s.servoPwmLimit != _pwm_limit){
        // Pause: to rest, change the limit with the torque off, and stand up again
        ESP_LOGI("HK", "Changing the PWM limit to %d: going to rest", s.servoPwmLimit);
        _transition = true;
        this->set_servo_motion(0, 1600);
        this->go_to_rest_pose();
        this->apply_pwm_limit(s.servoPwmLimit);
        this->stand_up();
        _servo_profile_ms = _servo_p_gain = -1; // written again just below
        _transition = false;
        target.setZero();
        next = esp_timer_get_time();
      }
      if(_running && (s.servoProfileMs != _servo_profile_ms || s.servoPGain != _servo_p_gain)){
        _servo_profile_ms = s.servoProfileMs;
        _servo_p_gain = s.servoPGain;
        this->set_servo_motion(_servo_profile_ms, _servo_p_gain);
      }
      ESP_LOGI("HK", "Settings: eps_A %g, eps_C %g, creativity %g, harmony %g, damping %g, causeaware %g, "
               "steps_delay %d, steps_averaging %d, amplitude %g, motor_smoothing %g, sensor_noise %g, "
               "motor_noise %g", p.epsA, p.epsC, p.creativity, p.harmony, p.damping, p.causeaware, p.steps4Delay,
               p.steps4Averaging, amplitude, smoothing, sensor_noise, motor_noise);
    }

    if(_running){
      // Sensors: the 12 joint positions; motors: the 12 joint targets; both normalised to the range
      this->read_normalised(sensors);
      for(size_t i = 0; i < NB_MOTORS; i++)
        x(i) = sensors(i) + noise(sensor_noise);
      int64_t t0 = esp_timer_get_time();
      const Homeokinesis_t::MotorVec& y = _hk.step(x);
      int64_t step_us = esp_timer_get_time() - t0;
      Homeokinesis_t::MotorVec y_noisy;
      for(size_t i = 0; i < NB_MOTORS; i++)
        y_noisy(i) = std::clamp(y(i) + noise(motor_noise), -1.0f, 1.0f);
      target += (y_noisy * amplitude - target) * (1.0f - smoothing);
      this->send_normalised(target);  // clipped to the range

      // A motor that trips (e.g. overload) goes limp: stop rather than carry on with a dead joint.
      // A garbled reply can look like an error, so stop only on a valid value read twice.
      size_t index = fault_check;
      int id = DXL_IDs[index];
      fault_check = (fault_check + 1) % NB_MOTORS;
      int32_t error = this->read_hardware_error(id);
      if(error > 0)
        error = (this->read_hardware_error(id) == error) ? error : 0;
      if(error == 0x20 && auto_recover){
        // Overload only: bring the motor back unless it keeps tripping (3 times in a minute)
        int64_t now = esp_timer_get_time();
        trips[index][2] = trips[index][1];
        trips[index][1] = trips[index][0];
        trips[index][0] = now;
        bool too_often = trips[index][2] != 0 && now - trips[index][2] < 60000000;
        if(!too_often){
          ESP_LOGW("HK", "Motor %d overload: rebooting it and continuing", id);
          this->add_to_motor_report("motor " + std::to_string(id) + " overload (recovered); ");
          this->recover_motor(id, _servo_profile_ms, _servo_p_gain);
          next = esp_timer_get_time();
          continue;
        }
        ESP_LOGW("HK", "Motor %d overloaded 3 times in a minute", id);
      }
      if(error > 0){
        ESP_LOGW("HK", "Motor %d hardware error 0x%02lx%s: stopping", id, (long) error, (error & 0x20) ? " (overload)" : "");
        this->add_to_motor_report("homeokinesis stopped: motor " + std::to_string(id) + " hardware error; ");
        _transition = true;
        this->stop_running("motor fault");
        _transition = false;
        char msg[40];
        snprintf(msg, sizeof(msg), "Motor %d fault: stopped", id);
        this->tft_status(msg, ST77XX_RED);
        next = esp_timer_get_time();
        continue;
      }

      step_us_sum += step_us;
      step_us_max = std::max(step_us_max, step_us);
      if(++steps_in_log == CONFIG_CONTROL_FREQUENCY){
        ESP_LOGI("HK", "step %ld: controller %lld us (max %lld), |y| %.2f, |C| %.2f, |A| %.2f, |error| %.3f, "
                 "incomplete reads %d", _hk.steps(), step_us_sum / steps_in_log, step_us_max, y.norm(),
                 _hk.C().norm(), _hk.A().norm(), _hk.prediction_error().norm(), this->incomplete_reads());
        step_us_sum = step_us_max = 0;
        steps_in_log = 0;
        this->reset_incomplete_reads();
      }
    }

    next += PERIOD;
    int64_t wait = next - esp_timer_get_time();
    if(wait > 0)
      std::this_thread::sleep_for(std::chrono::microseconds(wait));
    else
      next = esp_timer_get_time(); // late: don't try to catch up
  }
}

// ----- Menu ----- //

std::vector<Qutee::MenuItem> QuteeHomeokinesis::app_menu_items(){
  if(_transition || _run_request != -1)
    return {{"Please wait...", ST77XX_WHITE}};
  if(_running)
    return {{"Stop homeokinesis", ST77XX_RED}};
  return {{"Start homeokinesis", ST77XX_YELLOW}};
}

void QuteeHomeokinesis::on_app_menu_item(size_t index){
  (void) index;
  if(_transition || _run_request != -1)
    return;
  this->request_running(!_running);
}

// ----- ROS 2 parameters ----- //
//   ros2 param set /<robot name>/qutee_node eps_A 0.05
// running: start or stop the controller; reset: back to its initial state (models and controller).

void QuteeHomeokinesis::ros_setup(rcl_node_t* node, rclc_executor_t* executor, rclc_parameter_server_t* params){
  (void) node;
  (void) executor;
  HkSettings s = this->settings();
  const struct { const char* name; float value; } doubles[] = {
    {"eps_A", s.epsA}, {"eps_C", s.epsC}, {"creativity", s.creativity}, {"harmony", s.harmony},
    {"damping", s.damping}, {"causeaware", s.causeaware}, {"amplitude", s.amplitude},
    {"motor_smoothing", s.motorSmoothing}, {"sensor_noise", s.sensorNoise}, {"motor_noise", s.motorNoise}};
  for(const auto& d : doubles){
    RCCHECK(rclc_add_parameter(params, d.name, RCLC_PARAMETER_DOUBLE));
    RCCHECK(rclc_parameter_set_double(params, d.name, d.value));
  }
  const struct { const char* name; int value; } ints[] = {
    {"steps_delay", s.steps4Delay}, {"steps_averaging", s.steps4Averaging}, {"servo_profile_ms", s.servoProfileMs},
    {"servo_p_gain", s.servoPGain}, {"servo_pwm_limit", s.servoPwmLimit}};
  for(const auto& i : ints){
    RCCHECK(rclc_add_parameter(params, i.name, RCLC_PARAMETER_INT));
    RCCHECK(rclc_parameter_set_int(params, i.name, i.value));
  }
  const struct { const char* name; bool value; } bools[] = {
    {"auto_recover", s.autoRecover}, {"running", false}, {"reset", false}};
  for(const auto& b : bools){
    RCCHECK(rclc_add_parameter(params, b.name, RCLC_PARAMETER_BOOL));
    RCCHECK(rclc_parameter_set_bool(params, b.name, b.value));
  }
}

bool QuteeHomeokinesis::ros_parameter_changed(const Parameter* param){
  const char* name = param->name.data;
  const ParameterValue& value = param->value;

  if(strcmp(name, "running") == 0){
    this->request_running(value.bool_value);
    _running_published = value.bool_value;
    return true;
  }
  if(strcmp(name, "reset") == 0){
    if(value.bool_value)
      this->request_reset();
    return true;
  }
  HkSettings s = this->settings();
  if(strcmp(name, "auto_recover") == 0){
    s.autoRecover = value.bool_value;
    this->set_settings(s);
    return true;
  }
  if(value.type == rcl_interfaces__msg__ParameterType__PARAMETER_INTEGER){
    struct { const char* name; int* field; int64_t min, max; } ints[] = {
      {"steps_delay", &s.steps4Delay, 1, 9}, {"steps_averaging", &s.steps4Averaging, 1, 9},
      {"servo_profile_ms", &s.servoProfileMs, 0, 1000}, {"servo_p_gain", &s.servoPGain, 0, 16383},
      {"servo_pwm_limit", &s.servoPwmLimit, 0, 885}};
    bool found = false;
    for(auto& p : ints){
      if(strcmp(name, p.name) != 0)
        continue;
      found = true;
      if(value.integer_value < p.min || value.integer_value > p.max){
        ESP_LOGW("HK", "%s must be between %lld and %lld", name, p.min, p.max);
        return false;
      }
      *p.field = (int) value.integer_value;
    }
    if(!found)
      return false;
  } else {
    float v = (float) value.double_value;
    bool noise = strcmp(name, "sensor_noise") == 0 || strcmp(name, "motor_noise") == 0;
    if(v < 0 || (strcmp(name, "amplitude") == 0 && v > 1) || (strcmp(name, "motor_smoothing") == 0 && v > 0.99f)
       || (noise && v > 1)){
      ESP_LOGW("HK", "%s must be >= 0 (amplitude and noise: 0 to 1, motor_smoothing: 0 to 0.99)", name);
      return false;
    }
    if(strcmp(name, "eps_A") == 0) s.epsA = v;
    else if(strcmp(name, "eps_C") == 0) s.epsC = v;
    else if(strcmp(name, "creativity") == 0) s.creativity = v;
    else if(strcmp(name, "harmony") == 0) s.harmony = v;
    else if(strcmp(name, "damping") == 0) s.damping = v;
    else if(strcmp(name, "causeaware") == 0) s.causeaware = v;
    else if(strcmp(name, "amplitude") == 0) s.amplitude = v;
    else if(strcmp(name, "motor_smoothing") == 0) s.motorSmoothing = v;
    else if(strcmp(name, "sensor_noise") == 0) s.sensorNoise = v;
    else if(strcmp(name, "motor_noise") == 0) s.motorNoise = v;
    else return false;
  }
  this->set_settings(s);
  return true;
}

// Keep the "running" parameter in step with the controller: started or stopped from the menu, refused
// (motors busy) or stopped by a motor fault. Not while it is changing.
void QuteeHomeokinesis::ros_spin(rclc_parameter_server_t* params){
  if(_transition || _run_request != -1)
    return;
  bool running = _running;
  if(running != _running_published && rclc_parameter_set_bool(params, "running", running) == RCL_RET_OK)
    _running_published = running;
}
