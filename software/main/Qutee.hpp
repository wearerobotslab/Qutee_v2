#ifndef QUTEE_QUTEE_HPP_
#define QUTEE_QUTEE_HPP_

#define NN_INPUT_SIZE 18 // 6+12
#define NN_OUTPUT_SIZE 12
#define NB_STEPS floor(CONFIG_EPISODE_DURATION * CONFIG_CONTROL_FREQUENCY)
#define PERIOD floor(1000000 / CONFIG_CONTROL_FREQUENCY)

#undef HIGH // undef macro from adruino lib
#undef LOW // undef macro from arduino lib
#include "NNQuteeController.hpp"
#undef HIGH // undef macro from adruino lib
#undef LOW // undef macro from arduino lib


#include <atomic>
#include <iostream>
#include <mutex>
#include <string>
#include <sstream>
#include <vector>

// Homeokinesis controller, from the intrinsic_motivations submodule (components/intrinsic_motivations)
#include "homeokinesis.hpp"


#include "nvs_flash.h"
#include "nvs.h"
#define STORAGE_NAMESPACE "storage"

#include "Arduino.h"
#include <Wire.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789
#include "Adafruit_MAX1704X.h"

#include "Dynamixel2Arduino.h"
#include "utility/port_handler.h"
#include "QuteeDxlPortHandler.hpp"
//#include "QuteeController.hpp"

//This namespace is required to use Control table item names
using namespace ControlTableItem;
 

class Qutee
{
public:
  typedef NNQuteeController<NN_INPUT_SIZE, CONFIG_NB_HIDDEN_LAYERS, CONFIG_NB_NEURONS_PER_LAYER,NN_OUTPUT_SIZE> Policy_t;
  typedef Eigen::TensorFixedSize< float , Eigen::Sizes<NN_INPUT_SIZE, 1> > State_t;
  typedef Eigen::TensorFixedSize< float , Eigen::Sizes<NN_OUTPUT_SIZE, 1> > Actions_t;
  typedef std::vector<State_t,Eigen::aligned_allocator<State_t> > List_State_t;
  typedef std::vector<Actions_t,Eigen::aligned_allocator<Actions_t> > List_Actions_t;
  
  const float DEG_2_RAD = 0.01745329251f; //trig functions require radians, BNO055 outputs degrees
  
  Qutee():
    _states(NB_STEPS),
    _actions(NB_STEPS),
    _dxl_port(UART_NUM_1),
    _tft(TFT_CS, TFT_DC, TFT_RST),
    _bno(55, 0x28)
  {        
    
  }
  void run_episode();
  void init();
  void scan();
  void control_step(State_t& state_to_fill, Actions_t& actions_to_fill);   
  void tft_update_data_screen(const State_t& state, const Actions_t& actions, float freq);
  Policy_t& get_policy(){return _policy;}
  const List_Actions_t& get_list_actions(){return _actions;}
  const List_State_t& get_list_states(){return _states;}
  void calibration();
  void go_to_rest_pose();
  void go_to_standing_pose();
  float battery_voltage();
  std::string get_name(){return this->name;}
  void checkup();
  enum class MenuChoice { Ros, Homeokinesis };
  MenuChoice menu();
  void enter_ros_mode();
  void tft_status(const char* msg, uint16_t color = ST77XX_WHITE);
  // Motor problems found during the last episode (empty if none), reported by the status service
  const std::string& get_motor_report(){return _motor_report;}
  void set_motor_report(const std::string& report){_motor_report = report;}
  // Turn the torque off in the rest pose (default from menuconfig). Off when the robot hangs on a stand:
  // there the legs fall freely when released, and the back-driven motors can trip (input voltage error).
  bool torque_off_at_rest() const {return _torque_off_at_rest;}
  void set_torque_off_at_rest(bool on){_torque_off_at_rest = on;}

  // ----- Homeokinesis ----- //
  // Settings that can be changed over ROS (parameters of the qutee_node) while the controller runs
  // Defaults: tuned on the robot (on the floor) on 2026-10-10; the controller's own defaults are in homeokinesis.hpp
  struct HkSettings {
    float epsA = 0.5f, epsC = 0.5f;             // learning rates of the model (A) and controller (C)
    float creativity = 0.1f, harmony = 0.1f, damping = 0.00001f, causeaware = 0.01f;
    int steps4Delay = 10, steps4Averaging = 10;  // in control steps (50 Hz), 1..9
    float amplitude = 0.8f;                      // motor targets = controller output * amplitude (1 = +-45 deg)
    float motorSmoothing = 0.0f;                 // 0 off; s: each step the target moves (1 - s) of the way to the output
    // White noise, uniform in [-n, n] (1 = 45 deg): homeokinesis amplifies fluctuations, and these servos are
    // too precise to provide them. Added to each joint reading, and to each output before amplitude.
    float sensorNoise = 0.05f;
    float motorNoise = 0.05f;
    // Servos: time (ms) to reach each new target (0 = as fast as possible; 20 = one control period, so the
    // joints move continuously instead of jumping and stopping 50 times a second), and position P gain
    int servoProfileMs = 20;
    int servoPGain = 400;  // the XL330's own default; rollouts and poses use 1600 (set_PID_gains)
    // Maximum output of each motor (PWM Limit, 0..885 = 0..100%): limits the load that leads to overloads.
    // An EEPROM item, so it is changed at rest with the torque off; it also applies while standing up.
    int servoPwmLimit = 700;  // about 79%; motors otherwise run at 885 (100%)
    // On an overload, reboot only that motor, let it cool down 2 s and continue (3 trips in a minute: stop)
    bool autoRecover = true;
  };
  // Runs the controller until the user holds the Boot button (back to the menu). Blocks.
  void run_homeokinesis();
  bool homeokinesis_active() const {return _hk_active;}    // in homeokinesis mode (running or stopped)
  bool homeokinesis_running() const {return _hk_running;}  // controller driving the motors
  void hk_request_running(bool running){_hk_run_request = running ? 1 : 0;}
  void hk_request_reset(){_hk_reset_request = true;}
  HkSettings hk_settings(){std::lock_guard<std::mutex> lock(_hk_lock); return _hk_settings;}
  void hk_set_settings(const HkSettings& s){std::lock_guard<std::mutex> lock(_hk_lock); _hk_settings = s; _hk_settings_changed = true;}
private:
  void send_actions(const Actions_t& actions);
  void move_slowly(const Actions_t& target, int duration_ms);
  void set_torque(bool on);
  void set_servo_motion(int profile_ms, int p_gain);
  void set_pwm_limit(int limit);
  void recover_motor(int id, int profile_ms, int p_gain);
  Actions_t legs_raised_pose(float angle_deg);
  void get_motor_positions(State_t& state_ref, size_t offset);
  void get_state(State_t& state_ref);
  void check_motor_power();
  int reboot_faulted_motors(bool log_healthy);
  int32_t read_hardware_error(int id);
  void init_motors();
  void set_PID_gains();
  void init_imu();
  void init_tft();
  void displaySensorStatus(void);
  void tft_init_data_screen();
  void tft_load_screen();
  void tft_init_checkup_screen();
  void tft_update_checkup_screen(const float* err);
  void init_NVS();
  void manage_imu_calibration_nvs(uint8_t* calibration_data, size_t size, bool write);
  void load_imu_calibration();
  void print_state();
  void set_tft_brightness(size_t value);
  std::vector<std::string>  get_name_list();
  void select_name();
  bool wait_button_press();
  int poll_button();  // non-blocking: 0 nothing, 1 short press (on release), 2 long press (while held)
  static void hk_task(void* robot);
  void hk_loop();
  void hk_screen();
  void name_memory(size_t * id, bool write);
  
  // ----- Attributes ---- //
  List_State_t _states;
  List_Actions_t _actions;
  
  Policy_t _policy;
  State_t _state;
  
  static const int DXL_IDs[];
  static const uint8_t DXL_ID_CNT = 12;
  QuteeDxlPortHandler _dxl_port;
  Dynamixel2Arduino _dxl;
  Adafruit_ST7789 _tft;
  Adafruit_BNO055 _bno;
  bool _imu_available = false; // false when no BNO055 is detected at init; IMU readings are then zeros
  Adafruit_MAX17048 _maxlipo;
  bool _battery_monitor_available = false; // false when no MAX17048 is detected at init; battery_voltage() then returns 0
  
  
  
  // Starting address of the Data to read; Present Position = 132
  static const uint16_t SR_START_ADDR = 132;
  // Length of the Data to read; Length of Position data of X series is 4 byte
  static const uint16_t SR_ADDR_LEN = 4;
  // Starting address of the Data to write; Goal Position = 116
  static const uint16_t SW_START_ADDR = 116;
  // Length of the Data to write; Length of Position data of X series is 4 byte
  static const uint16_t SW_ADDR_LEN = 4;
  typedef struct sr_data{
    int32_t present_position;
  } __attribute__((packed)) sr_data_t;
  typedef struct sw_data{
    int32_t goal_position;
  } __attribute__((packed)) sw_data_t;
  sr_data_t _sr_data[DXL_ID_CNT];
  DYNAMIXEL::InfoSyncReadInst_t _sr_infos;
  DYNAMIXEL::XELInfoSyncRead_t _info_xels_sr[DXL_ID_CNT];
  sw_data_t _sw_data[DXL_ID_CNT];
  DYNAMIXEL::InfoSyncWriteInst_t _sw_infos;
  DYNAMIXEL::XELInfoSyncWrite_t _info_xels_sw[DXL_ID_CNT];
  static const uint16_t _user_pkt_buf_cap = 128;
  uint8_t _user_pkt_buf[_user_pkt_buf_cap];
  // Last position read from each motor (scaled like the state, 0 = neutral). Used when a
  // SyncRead stops early, so the state never contains uninitialised values.
  float _last_positions[DXL_ID_CNT] = {};
  int _incomplete_reads = 0; // SyncReads in the current episode that didn't get all motors
  bool _torque_on = false;
#ifdef CONFIG_TORQUE_OFF_AT_REST
  std::atomic<bool> _torque_off_at_rest{true};
#else
  std::atomic<bool> _torque_off_at_rest{false};
#endif
  std::string _motor_report;
  std::recursive_mutex _tft_lock;  // the screen is used by the homeokinesis and ROS tasks

  typedef im::hk::Homeokinesis<float, DXL_ID_CNT, DXL_ID_CNT> Homeokinesis_t;
  Homeokinesis_t _hk;
  std::mutex _hk_lock;  // guards _hk_settings, set from the ROS task
  HkSettings _hk_settings;
  std::atomic<bool> _hk_settings_changed{true};
  std::atomic<bool> _hk_active{false}, _hk_running{false}, _hk_reset_request{false};
  std::atomic<int> _hk_run_request{-1};  // -1 none, 0 stop, 1 start
  SemaphoreHandle_t _hk_done = nullptr;  // given by the homeokinesis task when it ends
  bool _btn_down = false, _btn_long = false;
  int64_t _btn_start = 0;
  std::string name;
};


#endif
