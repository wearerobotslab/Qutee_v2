#ifndef QUTEE_QUTEE_HPP_
#define QUTEE_QUTEE_HPP_

// The Qutee robot: the base of every application (software/apps/*).
//
// It owns the hardware (12 Dynamixel motors, IMU, battery monitor, screen, Boot button) and brings the robot
// up: screen, motors, a slow move to the rest pose, WiFi, then ROS (node /<name>/qutee_node, retrying until
// the agent answers). Then it runs the menu on the screen (select name, checkup, IMU calibration, plus the
// application's own entries) until the robot is switched off.
//
// An application derives from Qutee, adds its controller and its ROS services and parameters through the
// hooks below, and starts it with run() from its own app_main().

#define PERIOD (1000000 / CONFIG_CONTROL_FREQUENCY) // control period, in us

// Eigen before the Arduino headers: Arduino's HIGH and LOW macros break Eigen
#undef HIGH
#undef LOW
#include <eigen3/Eigen/Eigen>
#include <eigen3/unsupported/Eigen/CXX11/Tensor>
#undef HIGH
#undef LOW

#include <atomic>
#include <mutex>
#include <string>
#include <sstream>
#include <vector>

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

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rclc_parameter/rclc_parameter.h>

//This namespace is required to use Control table item names
using namespace ControlTableItem;


class Qutee
{
public:
  static constexpr int NB_MOTORS = 12;
  static constexpr int STATE_SIZE = 6 + NB_MOTORS; // orientation (3) and linear acceleration (3), then the joints
  static const int DXL_IDs[NB_MOTORS];             // leg x 10 + joint, as on the robot's body
  // State: see get_state(). Actions: one target per motor, 1 = 45 deg from the standing pose.
  typedef Eigen::TensorFixedSize< float , Eigen::Sizes<STATE_SIZE, 1> > State_t;
  typedef Eigen::TensorFixedSize< float , Eigen::Sizes<NB_MOTORS, 1> > Actions_t;

  const float DEG_2_RAD = 0.01745329251f; //trig functions require radians, BNO055 outputs degrees

  // app_name is shown at boot. ros_client_key_salt: each application uses its own micro-ROS client key,
  // so the agent doesn't mix up the entities of two firmwares flashed one after the other.
  Qutee(const char* app_name, uint32_t ros_client_key_salt);
  virtual ~Qutee() = default;
  static Qutee* instance(){return _instance;} // the robot (there is one per firmware), for C callbacks

  // Brings the robot up (hardware, rest pose, WiFi, ROS, the application's start()) and runs the menu.
  // Never returns.
  void run();

  // ----- Motors, sensors and poses ----- //
  // Each motor's position, scaled like the actions (0 = standing pose, 1 = 45 deg)
  void get_motor_positions(State_t& state_ref, size_t offset);
  // IMU (zeros without one) and joint positions
  void get_state(State_t& state_ref);
  void send_actions(const Actions_t& actions);
  void set_torque(bool on);
  void move_slowly(const Actions_t& target, int duration_ms);
  Actions_t legs_raised_pose(float angle_deg);
  void go_to_rest_pose();
  void go_to_standing_pose();
  void init_motors();
  void set_PID_gains();
  void set_servo_motion(int profile_ms, int p_gain);
  void set_pwm_limit(int limit);
  void recover_motor(int id, int profile_ms, int p_gain);
  int reboot_faulted_motors(bool log_healthy);
  int32_t read_hardware_error(int id);
  float battery_voltage();
  // SyncReads since the last reset that didn't get all motors (missing positions keep their last value)
  int incomplete_reads() const {return _incomplete_reads;}
  void reset_incomplete_reads(){_incomplete_reads = 0;}

  // Only one activity (a rollout, the checkup, a controller...) may move the motors at a time.
  // acquire_motors() returns false, and changes nothing, when another activity has them. (While the motors
  // are idle, the ROS task holds them for a few ms at a time to read the joint positions: acquire_motors()
  // then waits for it.)
  bool acquire_motors(const char* activity);
  void release_motors();
  const char* motors_owner() const {return _motors_owner.load();} // nullptr when free

  // Turn the torque off in the rest pose (default from menuconfig). Off when the robot hangs on a stand:
  // there the legs fall freely when released, and the back-driven motors can trip (input voltage error).
  bool torque_off_at_rest() const {return _torque_off_at_rest;}
  void set_torque_off_at_rest(bool on){_torque_off_at_rest = on;}

  // Motor problems since the last time it was cleared (empty if none), reported by the status service
  const std::string& get_motor_report(){return _motor_report;}
  void set_motor_report(const std::string& report){_motor_report = report;}
  void add_to_motor_report(const std::string& text){_motor_report += text;}

  // ----- Screen ----- //
  // One-line messages at the bottom of the screen: the status line, and the ROS line above it
  void tft_status(const char* msg, uint16_t color = ST77XX_WHITE);
  void tft_ros_status(const char* msg, uint16_t color = ST77XX_WHITE);
  std::string get_name(){return this->name;}
  bool ros_ready() const {return _ros_ready;}

protected:
  // ----- Hooks for the applications ----- //
  // Called once WiFi is up and the ROS task has been started (ROS may still be connecting): e.g. start a
  // controller task. Runs on the main task, before the menu.
  virtual void start() {}

  // Menu entries of the application, shown above the robot's own. Labels can change (e.g. Start/Stop):
  // the menu is redrawn when they do. on_app_menu_item() runs on the main task.
  struct MenuItem { std::string label; uint16_t color; };
  virtual std::vector<MenuItem> app_menu_items() {return {};}
  virtual void on_app_menu_item(size_t index) {(void) index;}

  // ROS, on the ROS task. ros_setup() creates the application's services and parameters: it may add up to
  // ros_handles() handles to the executor and ros_parameters() parameters to the parameter server.
  virtual size_t ros_handles() {return 0;}
  virtual size_t ros_parameters() {return 0;}
  virtual void ros_setup(rcl_node_t* node, rclc_executor_t* executor, rclc_parameter_server_t* params)
  {(void) node; (void) executor; (void) params;}
  // A parameter that the robot doesn't handle itself: return true to accept the new value
  virtual bool ros_parameter_changed(const Parameter* param) {(void) param; return false;}
  // Called by the ROS task about 20 times a second (parameters can only be changed from that task)
  virtual void ros_spin(rclc_parameter_server_t* params) {(void) params;}
  // Reported by the status service (number of weights of the policy, 0 when there is none)
  virtual uint16_t number_weights() {return 0;}

  // ----- Buttons ----- //
  // Non-blocking: NEXT is a short press (on release), SELECT a long press (as soon as it is long).
  // On the Reverse TFT Feather's three buttons: D0 previous, D1 select, D2 next.
  enum Button { BUTTON_NONE, BUTTON_NEXT, BUTTON_SELECT, BUTTON_PREVIOUS };
  Button poll_button();
  void wait_button_release();

  Adafruit_ST7789 _tft;
  std::recursive_mutex _tft_lock;  // the screen is used by several tasks

private:
  void init();
  void init_imu();
  void init_tft();
  void tft_load_screen();
  void scan();
  void check_motor_power();
  void displaySensorStatus(void);
  void init_NVS();
  void manage_imu_calibration_nvs(uint8_t* calibration_data, size_t size, bool write);
  void load_imu_calibration();
  void calibration();
  void checkup();
  void tft_init_checkup_screen();
  void tft_update_checkup_screen(const float* err);
  std::vector<std::string> get_name_list();
  void select_name();
  void name_memory(size_t * id, bool write);
  void menu_loop();
  void draw_menu(const std::vector<MenuItem>& items, size_t highlighted);
  void start_wifi();

  // ROS (on its own task)
  static void ros_task(void* arg);
  void ros_loop();
  static bool on_parameter_changed(const Parameter* old_param, const Parameter* new_param, void* context);
  static void status_callback(const void* req, void* res);
  void latest_positions(float* positions); // as last read, by any task; reads them when the motors are idle
  // On the ROS task only: publish joint_states if 50 ms have passed. Called by its loop, and after each motor
  // read made on the ROS task, so the topic continues during service callbacks that move the robot (rollouts).
  void publish_joint_states_if_due();
  TaskHandle_t _ros_task_handle = nullptr;
  bool _joint_publisher_ready = false;
  bool _time_synced = false;
  int64_t _next_joint_publish = 0;

  static Qutee* _instance;
  const char* _app_name;
  uint32_t _ros_client_key_salt;
  std::atomic<bool> _ros_ready{false};

  QuteeDxlPortHandler _dxl_port;
  Dynamixel2Arduino _dxl;
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
  sr_data_t _sr_data[NB_MOTORS];
  DYNAMIXEL::InfoSyncReadInst_t _sr_infos;
  DYNAMIXEL::XELInfoSyncRead_t _info_xels_sr[NB_MOTORS];
  sw_data_t _sw_data[NB_MOTORS];
  DYNAMIXEL::InfoSyncWriteInst_t _sw_infos;
  DYNAMIXEL::XELInfoSyncWrite_t _info_xels_sw[NB_MOTORS];
  static const uint16_t _user_pkt_buf_cap = 128;
  uint8_t _user_pkt_buf[_user_pkt_buf_cap];
  // Last position read from each motor (scaled like the state, 0 = neutral). Used when a
  // SyncRead stops early, so the state never contains uninitialised values.
  float _last_positions[NB_MOTORS] = {};
  // A copy for the joint_states topic, taken at each read (any task) and published by the ROS task
  std::mutex _published_positions_lock;
  float _published_positions[NB_MOTORS] = {};
  int64_t _published_positions_time = 0; // esp_timer time of the read, in us
  int _incomplete_reads = 0;
  bool _torque_on = false;
#ifdef CONFIG_TORQUE_OFF_AT_REST
  std::atomic<bool> _torque_off_at_rest{true};
#else
  std::atomic<bool> _torque_off_at_rest{false};
#endif
  std::atomic<const char*> _motors_owner{nullptr};
  std::string _motor_report;

  // Button state, for poll_button()
  bool _btn_down = false, _btn_long = false;
  int64_t _btn_start = 0;
  int _btn_last_extra = 0; // three-button boards: D1 and D2 as last seen

  std::string name;
};


#endif
