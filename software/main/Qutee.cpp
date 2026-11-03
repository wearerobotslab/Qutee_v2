#include "Qutee.hpp"
#include <inttypes.h>
#include <string.h>
#include <thread>
#include "driver/ledc.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <uros_network_interfaces.h>
#include <rmw_microros/rmw_microros.h>
#include "qutee_interface/srv/status.h"
#include <sensor_msgs/msg/joint_state.h>
#include "udp_log.hpp"

#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL            LEDC_CHANNEL_0
#define LEDC_DUTY_RES           LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
#define LEDC_FREQUENCY          (4000) // Frequency in Hertz. Set frequency at 4 kHz
#define TFT_BRIGHTNESS          0.5 // backlight duty cycle, from 0 (off) to 1 (full)

// Screen rows (240 x 135 pixels): the menu at the top, then the ROS line and the status line (text size 1)
#define ROS_ROW    112
#define STATUS_ROW 124
#define MENU_HEIGHT 108

// Buttons: the Reverse TFT Feather has D0 (active low), D1 and D2 (active high).
// The regular ESP32-S3 TFT Feather only has D0 (Boot), so its menus use short presses (next) and long presses (select).
#if TFT_I2C_POWER == 7 // adafruit_feather_esp32s3_reversetft
#define QUTEE_THREE_BUTTONS 1
#else
#define QUTEE_THREE_BUTTONS 0
#endif
#define LONG_PRESS_US 700000

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ESP_LOGE("UROS","Failed status on line %d: %d. Aborting.",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ESP_LOGW("UROS","Failed status on line %d: %d. Continuing.",__LINE__,(int)temp_rc);}}

const int Qutee::DXL_IDs[] = {11, 12, 13, 21, 22, 23,  31, 32, 33, 41, 42, 43};
Qutee* Qutee::_instance = nullptr;

Qutee::Qutee(const char* app_name, uint32_t ros_client_key_salt):
  _tft(TFT_CS, TFT_DC, TFT_RST),
  _app_name(app_name),
  _ros_client_key_salt(ros_client_key_salt),
  _dxl_port(UART_NUM_1),
  _bno(55, 0x28)
{
  _instance = this;
}

// ----- Start-up ----- //

void Qutee::run(){
  udp_log_init(); // keep the logs from now on, to send them over WiFi once it is connected
  this->init();
  this->start_wifi();
  // The ROS task retries until the agent answers, so the robot is usable (menu, checkup) without it
  xTaskCreate(ros_task, "uros_task", CONFIG_MICRO_ROS_APP_STACK, this, CONFIG_MICRO_ROS_APP_TASK_PRIO, NULL);
  this->start();
  this->menu_loop();
}

void Qutee::init(){
  init_tft();

  size_t name_id = 0; // first name in the list until one is selected in the menu (the NVS read fails on a new robot)
  this->name_memory(&name_id,false);
  std::vector<std::string> names = get_name_list();
  if(name_id < names.size())
    this->name = names[name_id];
  else
    this->name = "No-Name";

  tft_load_screen();
  _dxl.setPort(_dxl_port);
  int32_t baud = 3000000;
  int8_t  protocol = 2;
  _dxl.setPortProtocolVersion((float) protocol);
  ESP_LOGI("DXL: ","PROTOCOL %i", protocol);
  ESP_LOGI("DXL: ","BAUDRATE %ld", baud);
  _dxl.begin(baud);

  // Fill the members of structure to syncRead using external user packet buffer
  _sr_infos.packet.p_buf = _user_pkt_buf;
  _sr_infos.packet.buf_capacity = _user_pkt_buf_cap;
  _sr_infos.packet.is_completed = false;
  _sr_infos.addr = SR_START_ADDR;
  _sr_infos.addr_length = SR_ADDR_LEN;
  _sr_infos.p_xels = _info_xels_sr;
  _sr_infos.xel_count = 0;

  for(int i = 0; i < NB_MOTORS; i++){
    _info_xels_sr[i].id = DXL_IDs[i];
    _info_xels_sr[i].p_recv_buf = (uint8_t*)&_sr_data[i];
    _sr_infos.xel_count++;
  }
  _sr_infos.is_info_changed = true;

  // Fill the members of structure to syncWrite using internal packet buffer
  _sw_infos.packet.p_buf = nullptr;
  _sw_infos.packet.is_completed = false;
  _sw_infos.addr = SW_START_ADDR;
  _sw_infos.addr_length = SW_ADDR_LEN;
  _sw_infos.p_xels = _info_xels_sw;
  _sw_infos.xel_count = 0;

  for(int i = 0; i < NB_MOTORS; i++){
    _info_xels_sw[i].id = DXL_IDs[i];
    _info_xels_sw[i].p_data = (uint8_t*)&_sw_data[i].goal_position;
    _sw_infos.xel_count++;
  }
  _sw_infos.is_info_changed = true;

  init_imu();

  // init battery monitor
  if (this->_maxlipo.begin()) {
    this->_battery_monitor_available = true;
    ESP_LOGI("BATTERY: ","Found MAX17048 with Chip ID: %d",  this->_maxlipo.getChipID());
    this->_maxlipo.reset();
  }
  else {
    ESP_LOGW("BATTERY: ","Couldnt find Adafruit MAX17048, continuing without battery monitor (battery voltage will read as 0)");
    this->tft_status("No battery monitor, continuing", ST77XX_YELLOW);
    delay(1000);
  }

  check_motor_power();
  this->tft_status("Setting up motors...");
  init_motors();
  scan();
  // Settle slowly: raise the legs so the torso rests on the ground, then lower them to the rest pose
  this->tft_status("Moving to the rest pose...");
  move_slowly(legs_raised_pose(CONFIG_LEG_UP_ANGLE), CONFIG_POSE_TRANSITION_TIME_MS);
  go_to_rest_pose();
}

void Qutee::init_imu(){

  // I2C pins and power pin come from the board variant (CONFIG_ARDUINO_VARIANT)
  Wire.setPins(SDA, SCL);
  Wire.begin();

  // POWER UP I2C
  gpio_set_direction((gpio_num_t)TFT_I2C_POWER, GPIO_MODE_OUTPUT);
  gpio_set_level((gpio_num_t)TFT_I2C_POWER, 1);
  delay(10);
  // List the I2C devices that answer (BNO055 is 0x28/0x29, battery monitor is 0x36 for MAX17048 or 0x0b for LC709203F)
  int nb_i2c_devices = 0;
  for(uint8_t addr = 1; addr < 127; addr++){
    Wire.beginTransmission(addr);
    if(Wire.endTransmission() == 0){
      ESP_LOGI("I2C","Device found at 0x%02x", addr);
      nb_i2c_devices++;
    }
  }
  ESP_LOGI("I2C","%i device(s) found on SDA=GPIO%d SCL=GPIO%d", nb_i2c_devices, SDA, SCL);
  ESP_LOGI("IMU","START IMU PROCESS.");
  if (!this->_bno.begin())
    {
      ESP_LOGW("IMU","No BNO055 detected, continuing without IMU (orientation and acceleration will read as zero)");
      this->tft_status("IMU not found, continuing without it", ST77XX_YELLOW);
      delay(1000);
      return;
    }
  this->_imu_available = true;
  ESP_LOGI("IMU","Init imu Done");
  delay(1000);
  ESP_LOGI("IMU","Setting NDOF mode");
  this->load_imu_calibration();
  this->_bno.setMode(OPERATION_MODE_IMUPLUS);
  displaySensorStatus();
  if(!this->_bno.isFullyCalibrated())
    ESP_LOGW("IMU","The IMU is not calibrated: choose Calibrate IMU in the menu");
  ESP_LOGI("IMU","IMU DONE.");

}

void Qutee::init_tft(){
  gpio_set_direction((gpio_num_t)TFT_BACKLITE, GPIO_MODE_OUTPUT);
  // Prepare and then apply the LEDC PWM timer configuration (backlight brightness)
  ledc_timer_config_t ledc_timer;
  ledc_timer.speed_mode       = LEDC_MODE;
  ledc_timer.duty_resolution  = LEDC_DUTY_RES;
  ledc_timer.timer_num        = LEDC_TIMER;
  ledc_timer.freq_hz          = LEDC_FREQUENCY;  // Set output frequency at 4 kHz
  ledc_timer.clk_cfg          = LEDC_AUTO_CLK;
  ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

  // Prepare and then apply the LEDC PWM channel configuration
  ledc_channel_config_t ledc_channel;
  ledc_channel.speed_mode     = LEDC_MODE;
  ledc_channel.channel        = LEDC_CHANNEL;
  ledc_channel.timer_sel      = LEDC_TIMER;
  ledc_channel.intr_type      = LEDC_INTR_DISABLE;
  ledc_channel.gpio_num       = (gpio_num_t)TFT_BACKLITE;
  ledc_channel.duty           = TFT_BRIGHTNESS*8192;
  ledc_channel.hpoint         = 0;

  ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
  // turn on the TFT / I2C power supply
  gpio_set_direction((gpio_num_t)TFT_I2C_POWER, GPIO_MODE_OUTPUT);
  gpio_set_level((gpio_num_t)TFT_I2C_POWER, 1);

  delay(10);
  this->_tft.init(135, 240);
  this->_tft.setRotation(3);
  gpio_set_direction(GPIO_NUM_0, GPIO_MODE_INPUT);
#if QUTEE_THREE_BUTTONS
  gpio_set_direction(GPIO_NUM_1, GPIO_MODE_INPUT);
  gpio_set_direction(GPIO_NUM_2, GPIO_MODE_INPUT);
#endif
}

void Qutee::tft_load_screen() {
  std::lock_guard<std::recursive_mutex> lock(this->_tft_lock);
  this->_tft.setTextWrap(false);
  this->_tft.fillScreen(ST77XX_BLACK);
  this->_tft.setCursor(0, 20);
  this->_tft.setTextColor(ST77XX_GREEN);
  this->_tft.setTextSize(2);
  this->_tft.println("Hi! My name is");
  this->_tft.println(this->name.c_str());
  this->_tft.setTextColor(ST77XX_YELLOW);
  this->_tft.println(this->_app_name);
  this->_tft.setTextColor(ST77XX_RED);
  this->_tft.println("Loading!");
  this->tft_status("Powered on");
  delay(1000);
}

static void tft_line(Adafruit_ST7789& tft, int row, const char* msg, uint16_t color) {
  tft.fillRect(0, row, 240, 8, ST77XX_BLACK);
  tft.setTextWrap(false);
  tft.setTextSize(1);
  tft.setTextColor(color, ST77XX_BLACK);
  tft.setCursor(0, row);
  tft.print(msg);
}

void Qutee::tft_status(const char* msg, uint16_t color) {
  std::lock_guard<std::recursive_mutex> lock(this->_tft_lock);
  tft_line(this->_tft, STATUS_ROW, msg, color);
}

void Qutee::tft_ros_status(const char* msg, uint16_t color) {
  std::lock_guard<std::recursive_mutex> lock(this->_tft_lock);
  tft_line(this->_tft, ROS_ROW, msg, color);
}

// ----- Non-volatile storage: IMU calibration and the robot's name ----- //

void Qutee::init_NVS(){
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition was truncated and needs to be erased
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK( err );
}

void Qutee::manage_imu_calibration_nvs(uint8_t* calibration_data, size_t size, bool write = false){
  this->init_NVS();
  nvs_handle_t nvs_handle;
  esp_err_t err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &nvs_handle);
  if (err != ESP_OK) {
    ESP_LOGE("NVS STORAGE: ","Error (%s) opening NVS handle!", esp_err_to_name(err));
    return;
  }
  size_t required_size = size * sizeof(uint8_t);
  if(write){
    err = nvs_set_blob(nvs_handle, "imu_cal", calibration_data, required_size);
    if (err != ESP_OK) {ESP_LOGE("NVS STORAGE: ","Write Failed, error: %s",  esp_err_to_name(err));}
    err = nvs_commit(nvs_handle);
    if (err != ESP_OK) {ESP_LOGE("NVS STORAGE: ","Commit Failed, error: %s", esp_err_to_name(err));}
  }
  else{
    err = nvs_get_blob(nvs_handle, "imu_cal", calibration_data, &required_size);
    if (err != ESP_OK) {ESP_LOGW("NVS STORAGE: ","No IMU calibration read: %s", esp_err_to_name(err));}
  }
  nvs_close(nvs_handle);
}

void Qutee::load_imu_calibration()
{
  uint8_t calibration_data[23] = {}; // one more value, set to 42 as a sanity check: if it isn't 42, the data weren't saved properly
  this->manage_imu_calibration_nvs(calibration_data, 23, false);
  if(calibration_data[22] == 42)
    {
      ESP_LOGI("CALIBRATION DATA: "," successfully loaded saved values");
      this->_bno.setSensorOffsets(calibration_data);
    }
  else
    ESP_LOGI("CALIBRATION DATA: ","no existing calibration data. Calibration is required");
}

// Waits until the IMU is fully calibrated and still (gravity removed) for 1 s, then stores the calibration.
// A button press cancels it.
void Qutee::calibration()
{
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  wait_button_release();
  {
    std::lock_guard<std::recursive_mutex> lock(this->_tft_lock);
    this->_tft.fillRect(0, 0, 240, MENU_HEIGHT, ST77XX_BLACK);
  }
  this->tft_status("Press button to cancel");

  uint8_t count =0;
  sensors_event_t  linearAccelData;
  float accel_amp;
  while(count<10){ // fully calibrated and with low acceleration (i.e., learned to remove gravity) for 10 consecutive steps, i.e., one second.
    if(this->poll_button() != BUTTON_NONE){
      this->tft_status("IMU calibration cancelled", ST77XX_YELLOW);
      return;
    }
    this->_bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
    accel_amp = linearAccelData.acceleration.x*linearAccelData.acceleration.x + linearAccelData.acceleration.y*linearAccelData.acceleration.y + linearAccelData.acceleration.z*linearAccelData.acceleration.z;

    if(this->_bno.isFullyCalibrated() && accel_amp<0.01)
      count++;
    else
      count=0;
    this->_bno.getCalibration(&system, &gyro, &accel, &mag);
    ESP_LOGI("CALIBRATION: ","system %i, gyro %i, accel %i, mag %i, accel_amp %f",  system , gyro ,accel, mag, accel_amp);
    {
      std::lock_guard<std::recursive_mutex> lock(this->_tft_lock);
      this->_tft.setTextSize(1);
      this->_tft.setTextColor(ST77XX_BLUE,ST77XX_BLACK);
      this->_tft.setCursor(0, 10);
      this->_tft.print("system: "); this->_tft.println(system);
      this->_tft.print("gyro: "); this->_tft.println(gyro);
      this->_tft.print("accel: "); this->_tft.println(accel);
      this->_tft.print("mag: "); this->_tft.println(mag);
      this->_tft.print("Accel amplitude: "); this->_tft.println(accel_amp);
      this->_tft.print("x: "); this->_tft.print(linearAccelData.acceleration.x);
      this->_tft.print(" y: "); this->_tft.print(linearAccelData.acceleration.y);
      this->_tft.print(" z: "); this->_tft.println(linearAccelData.acceleration.z);
    }
    delay(100);
  }

  uint8_t calibration_data[23];
  if(this->_bno.getSensorOffsets(calibration_data)){
    calibration_data[22]=42;
    this->manage_imu_calibration_nvs(calibration_data, 23, true);
    this->tft_status("IMU calibration done and saved", ST77XX_GREEN);
  }
  else{
    ESP_LOGE("CALIBRATION: "," ERROR FAILED TO GET CALIBRATION PROFILE");
    this->tft_status("IMU calibration failed", ST77XX_RED);
  }
  this->_bno.setMode(OPERATION_MODE_IMUPLUS);
}

std::vector<std::string>  Qutee::get_name_list(){
  std::vector<std::string> names;
  std::stringstream ss(CONFIG_QUTEE_NAME);
  std::string name;

  while (std::getline(ss, name, ';')) {
    names.push_back(name);
  }
  return names;
}

void Qutee::name_memory(size_t * id, bool write = false){
  this->init_NVS();
  nvs_handle_t nvs_handle;
  esp_err_t err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &nvs_handle);
  if (err != ESP_OK) {
    ESP_LOGE("NVS STORAGE: ","Error (%s) opening NVS handle!", esp_err_to_name(err));
    return;
  }
  size_t required_size = 1 * sizeof(size_t);
  if(write){
    err = nvs_set_blob(nvs_handle, "name_id", id, required_size);
    if (err != ESP_OK) {ESP_LOGE("NVS STORAGE: ","Write Failed, error: %s",  esp_err_to_name(err));}
    err = nvs_commit(nvs_handle);
    if (err != ESP_OK) {ESP_LOGE("NVS STORAGE: ","Commit Failed, error: %s", esp_err_to_name(err));}
  }
  else{
    err = nvs_get_blob(nvs_handle, "name_id", id, &required_size);
    if (err != ESP_OK) {ESP_LOGW("NVS STORAGE: ","No robot name stored yet: %s", esp_err_to_name(err));}
  }
  nvs_close(nvs_handle);
}

// The name is the ROS namespace, which is set when ROS starts: a new name applies after a restart
void Qutee::select_name(){
  std::vector<std::string> names = this->get_name_list();
  size_t id = 0;
  for(size_t i = 0; i < names.size(); i++)
    if(names[i] == this->name)
      id = i;
  wait_button_release();
  bool redraw = true;
  while(true){
    if(redraw){
      std::lock_guard<std::recursive_mutex> lock(this->_tft_lock);
      this->_tft.fillRect(0, 0, 240, MENU_HEIGHT, ST77XX_BLACK);
      this->_tft.setTextSize(2);
      this->_tft.setCursor(0, 0);
      this->_tft.setTextColor(ST77XX_GREEN);
      this->_tft.println("Select Name");
      this->_tft.setCursor(0, 50);
      this->_tft.setTextColor(ST77XX_BLUE);
      this->_tft.print("> ");
      this->_tft.println(names[id].c_str());
      this->tft_status("Press: next name   Hold: select");
      redraw = false;
    }
    Button b = this->poll_button();
    if(b == BUTTON_SELECT)
      break;
    if(b == BUTTON_NEXT){
      id = (id + 1) % names.size();
      redraw = true;
    } else if(b == BUTTON_PREVIOUS){
      id = (id + names.size() - 1) % names.size();
      redraw = true;
    }
    delay(20);
  }
  if(names[id] != this->name){
    this->name_memory(&id,true);
    ESP_LOGI("Menu", "Name %s saved: it is used after a restart", names[id].c_str());
    this->tft_status("Name saved: restart to use it", ST77XX_YELLOW);
  } else {
    this->tft_status("Name unchanged");
  }
}

// ----- Buttons and menu ----- //

Qutee::Button Qutee::poll_button(){
  int64_t now = esp_timer_get_time();
#if QUTEE_THREE_BUTTONS
  // D1 and D2 are active high: act on the press
  int extra = (gpio_get_level(GPIO_NUM_1) ? 1 : 0) | (gpio_get_level(GPIO_NUM_2) ? 2 : 0);
  int pressed_now = extra & ~this->_btn_last_extra;
  this->_btn_last_extra = extra;
  if(pressed_now & 1) return BUTTON_SELECT;
  if(pressed_now & 2) return BUTTON_NEXT;
#endif
  bool pressed = !gpio_get_level(GPIO_NUM_0); // D0 is active low
  if(pressed && !this->_btn_down){
    this->_btn_down = true;
    this->_btn_long = false;
    this->_btn_start = now;
  } else if(pressed && !this->_btn_long && now - this->_btn_start > LONG_PRESS_US){
    this->_btn_long = true;
#if !QUTEE_THREE_BUTTONS
    return BUTTON_SELECT;
#endif
  } else if(!pressed && this->_btn_down){
    this->_btn_down = false;
    if(!this->_btn_long && now - this->_btn_start > 30000) // debounce
      return QUTEE_THREE_BUTTONS ? BUTTON_PREVIOUS : BUTTON_NEXT;
  }
  return BUTTON_NONE;
}

// Wait until no button is pressed, so the press that chose an entry isn't seen again
void Qutee::wait_button_release(){
#if QUTEE_THREE_BUTTONS
  while(!gpio_get_level(GPIO_NUM_0) || gpio_get_level(GPIO_NUM_1) || gpio_get_level(GPIO_NUM_2)) delay(10);
  this->_btn_last_extra = 0;
#else
  while(!gpio_get_level(GPIO_NUM_0)) delay(10);
#endif
  delay(50); // debounce
  this->_btn_down = false;
}

void Qutee::draw_menu(const std::vector<MenuItem>& items, size_t highlighted){
  std::lock_guard<std::recursive_mutex> lock(this->_tft_lock);
  this->_tft.fillRect(0, 0, 240, MENU_HEIGHT, ST77XX_BLACK);
  this->_tft.setTextWrap(false);
  this->_tft.setTextSize(2);
  int spacing = std::min(26, MENU_HEIGHT / (int) std::max<size_t>(1, items.size()));
  for(size_t i = 0; i < items.size(); i++){
    this->_tft.setCursor(0, i * spacing);
    this->_tft.setTextColor(items[i].color);
    this->_tft.print(i == highlighted ? "> " : "  ");
    this->_tft.println(items[i].label.c_str());
  }
}

// The application's entries first, then the robot's own
void Qutee::menu_loop(){
  enum { NAME, CHECKUP, CALIBRATION };
  std::vector<MenuItem> drawn;
  size_t highlighted = 0, drawn_highlight = (size_t) -1;
  wait_button_release();
  this->tft_status(QUTEE_THREE_BUTTONS ? "D0/D2: move   D1: select" : "Press: next   Hold: select");
  ESP_LOGI("Menu", "Menu ready");
  while(true){
    std::vector<MenuItem> items = this->app_menu_items();
    size_t nb_app = items.size();
    std::vector<int> robot_entries = {NAME, CHECKUP};
    items.push_back({"Select Name", ST77XX_GREEN});
    items.push_back({"Checkup", ST77XX_RED});
    if(this->_imu_available){
      items.push_back({"Calibrate IMU", ST77XX_CYAN});
      robot_entries.push_back(CALIBRATION);
    }
    highlighted = std::min(highlighted, items.size() - 1);

    bool changed = highlighted != drawn_highlight || items.size() != drawn.size();
    for(size_t i = 0; !changed && i < items.size(); i++)
      changed = items[i].label != drawn[i].label || items[i].color != drawn[i].color;
    if(changed){
      this->draw_menu(items, highlighted);
      drawn = items;
      drawn_highlight = highlighted;
    }

    Button b = this->poll_button();
    if(b == BUTTON_NEXT)
      highlighted = (highlighted + 1) % items.size();
    else if(b == BUTTON_PREVIOUS)
      highlighted = (highlighted + items.size() - 1) % items.size();
    else if(b == BUTTON_SELECT){
      if(highlighted < nb_app){
        this->on_app_menu_item(highlighted);
      } else {
        int entry = robot_entries[highlighted - nb_app];
        if(entry == NAME){
          this->select_name();
        } else if(!this->acquire_motors(entry == CHECKUP ? "checkup" : "IMU calibration")){
          char msg[48];
          snprintf(msg, sizeof(msg), "Busy: %s is running", this->motors_owner() ? this->motors_owner() : "?");
          this->tft_status(msg, ST77XX_YELLOW);
        } else {
          if(entry == CHECKUP)
            this->checkup();
          else
            this->calibration();
          this->release_motors();
        }
      }
      wait_button_release();
      drawn.clear(); // redraw
    }
    delay(20);
  }
}

// Held by the ROS task while it reads the joint positions of idle motors (a few ms)
static const char* const JOINT_READING = "joint reading";

bool Qutee::acquire_motors(const char* activity){
  for(int attempt = 0; ; attempt++){
    const char* expected = nullptr;
    if(this->_motors_owner.compare_exchange_strong(expected, activity))
      return true;
    if(expected != JOINT_READING || attempt >= 100){
      ESP_LOGW("Qutee", "%s refused: %s is using the motors", activity, expected);
      return false;
    }
    vTaskDelay(pdMS_TO_TICKS(1)); // the joint reading is about to end
  }
}

void Qutee::release_motors(){
  this->_motors_owner = nullptr;
}

// ----- WiFi and ROS ----- //

void Qutee::start_wifi(){
#if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
  this->tft_ros_status("Connecting to WiFi " CONFIG_ESP_WIFI_SSID);
  ESP_ERROR_CHECK(uros_network_interface_initialize());
  // uros_network_interface_initialize() also returns when the connection failed, so check for an IP address
  esp_netif_ip_info_t ip_info = {};
  esp_netif_t* netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
  if(netif && esp_netif_get_ip_info(netif, &ip_info) == ESP_OK && ip_info.ip.addr != 0){
    char msg[40];
    snprintf(msg, sizeof(msg), "WiFi connected, IP " IPSTR, IP2STR(&ip_info.ip));
    this->tft_ros_status(msg, ST77XX_GREEN);
    udp_log_start();
  }
  else
    this->tft_ros_status("WiFi failed: check SSID/password", ST77XX_RED);
#endif
}

void Qutee::ros_task(void* arg){
  static_cast<Qutee*>(arg)->ros_loop();
  vTaskDelete(NULL);
}

static rclc_parameter_server_t param_server;
static rcl_publisher_t joint_publisher;
static sensor_msgs__msg__JointState joint_state;
static double joint_positions[Qutee::NB_MOTORS];

bool Qutee::on_parameter_changed(const Parameter* old_param, const Parameter* new_param, void* context)
{
  (void) old_param;
  (void) context;
  if(new_param == NULL)
    return false; // parameters can't be deleted
  Qutee* robot = Qutee::instance();
  if(strcmp(new_param->name.data, "torque_off_at_rest") == 0){
    robot->set_torque_off_at_rest(new_param->value.bool_value);
    return true;
  }
  return robot->ros_parameter_changed(new_param);
}

void Qutee::status_callback(const void* req, void* res){
  (void) req;
  static char empty_message = '\0';
  qutee_interface__srv__Status_Response* res_in = (qutee_interface__srv__Status_Response*) res;
  Qutee* robot = Qutee::instance();
  ESP_LOGI("ROS: ","Service Status request received");
  res_in->battery = robot->battery_voltage();
  res_in->number_weights = robot->number_weights();
  // Motor problems found since the last rollout or controller start (empty if none)
  const std::string& report = robot->get_motor_report();
  res_in->error_message.data = report.empty() ? &empty_message : const_cast<char*>(report.c_str());
  res_in->error_message.size = report.size();
  res_in->error_message.capacity = report.size() + 1;
}

// Connects to the agent (retrying until it answers), creates the node /<name>/qutee_node with the status
// service and the parameter server, lets the application add its own, and spins forever.
void Qutee::ros_loop(){
  this->_ros_task_handle = xTaskGetCurrentTaskHandle();
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  RCCHECK(rcl_init_options_init(&init_options, allocator));
  RCCHECK(rcl_init_options_set_domain_id(&init_options, 0));
#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
  rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
  RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));
  // A client key that stays the same across resets (from the MAC address): the agent then replaces the
  // robot's previous session instead of keeping its stale services alongside the new ones
  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_WIFI_STA);
  uint32_t client_key = ((uint32_t)mac[2] << 24) | ((uint32_t)mac[3] << 16) | ((uint32_t)mac[4] << 8) | mac[5];
  RCCHECK(rmw_uros_options_set_client_key(client_key ^ this->_ros_client_key_salt, rmw_options));
#endif

  rclc_support_t support;
  this->tft_ros_status("Connecting to agent " CONFIG_MICRO_ROS_AGENT_IP ":" CONFIG_MICRO_ROS_AGENT_PORT);
  bool warned = false;
  while(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator) != RCL_RET_OK){
    if(!warned){
      ESP_LOGW("UROS","Could not reach the micro-ROS agent at " CONFIG_MICRO_ROS_AGENT_IP ":" CONFIG_MICRO_ROS_AGENT_PORT ", retrying");
      this->tft_ros_status("Agent unreachable, retrying...", ST77XX_RED);
      warned = true;
    }
    vTaskDelay(pdMS_TO_TICKS(2000));
  }

  rcl_node_t node;
  RCCHECK(rclc_node_init_default(&node, "qutee_node", this->name.c_str(), &support));

  rclc_parameter_options_t options = {};
  options.notify_changed_over_dds = false;
  options.max_params = 1 + this->ros_parameters();
  options.allow_undeclared_parameters = false;
  options.low_mem_mode = false;
  RCCHECK(rclc_parameter_server_init_with_option(&param_server, &node, &options));
  RCCHECK(rclc_add_parameter(&param_server, "torque_off_at_rest", RCLC_PARAMETER_BOOL));
  RCCHECK(rclc_parameter_set_bool(&param_server, "torque_off_at_rest", this->torque_off_at_rest()));

  rclc_executor_t executor;
  unsigned int num_handles = 1 + RCLC_EXECUTOR_PARAMETER_SERVER_HANDLES + this->ros_handles();
  RCCHECK(rclc_executor_init(&executor, &support.context, num_handles, &allocator));

  rcl_service_t service_status;
  static qutee_interface__srv__Status_Response res_status;
  static qutee_interface__srv__Status_Request req_status;
  RCCHECK(rclc_service_init_default(&service_status, &node, ROSIDL_GET_SRV_TYPE_SUPPORT(qutee_interface, srv, Status), "status"));
  RCCHECK(rclc_executor_add_service(&executor, &service_status, &req_status, &res_status, status_callback));

  // The application's services and parameters; the parameter server is added to the executor once all
  // parameters have their initial values
  this->ros_setup(&node, &executor, &param_server);
  RCCHECK(rclc_executor_add_parameter_server(&executor, &param_server, on_parameter_changed));

  // joint_states: the 12 joint positions, names "11".."43" (motor IDs), in radians from the standing pose
  RCCHECK(rclc_publisher_init_best_effort(&joint_publisher, &node,
                                          ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState), "joint_states"));
  static char joint_names[NB_MOTORS][4];
  static rosidl_runtime_c__String names[NB_MOTORS];
  static char empty_frame[] = "";
  for(int i = 0; i < NB_MOTORS; i++){
    snprintf(joint_names[i], sizeof(joint_names[i]), "%d", DXL_IDs[i]);
    names[i].data = joint_names[i];
    names[i].size = strlen(joint_names[i]);
    names[i].capacity = sizeof(joint_names[i]);
  }
  joint_state.name.data = names;
  joint_state.name.size = joint_state.name.capacity = NB_MOTORS;
  joint_state.position.data = joint_positions;
  joint_state.position.size = joint_state.position.capacity = NB_MOTORS;
  joint_state.velocity.size = joint_state.effort.size = 0;
  joint_state.header.frame_id.data = empty_frame;
  joint_state.header.frame_id.size = 0;
  joint_state.header.frame_id.capacity = sizeof(empty_frame);
  // Time stamps from the agent's clock when available, else the time since boot
  this->_time_synced = rmw_uros_sync_session(1000) == RMW_RET_OK;
  this->_next_joint_publish = esp_timer_get_time();
  this->_joint_publisher_ready = true;

  char msg[48];
  snprintf(msg, sizeof(msg), "ROS ready: /%s/qutee_node", this->name.c_str());
  ESP_LOGI("UROS", "%s (time %s)", msg, this->_time_synced ? "synced with the agent" : "since boot");
  this->tft_ros_status(msg, ST77XX_GREEN);
  this->_ros_ready = true;

  // Ping the agent regularly: without traffic from the robot, NAT between the robot and the
  // agent (e.g. Docker port forwarding) drops the UDP flow and service requests never arrive.
  const int64_t keepalive_period_us = 5000000;
  int64_t last_keepalive = esp_timer_get_time();
  while(true){
    // Soft check: a reply that fails to send (e.g. too large) must not stop the ROS task
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5)));
    this->ros_spin(&param_server);
    this->publish_joint_states_if_due();
    if(esp_timer_get_time() - last_keepalive > keepalive_period_us){
      if(rmw_uros_ping_agent(100, 1) != RMW_RET_OK)
        ESP_LOGW("UROS","No answer from the agent to the keepalive ping");
      last_keepalive = esp_timer_get_time();
    }
    int64_t wait_us = this->_next_joint_publish - esp_timer_get_time();
    vTaskDelay(pdMS_TO_TICKS(std::max<int64_t>(5, std::min<int64_t>(wait_us, 50000) / 1000)));
  }
}

void Qutee::publish_joint_states_if_due(){
  const int64_t joint_period_us = 50000; // 20 Hz
  int64_t now = esp_timer_get_time();
  if(now < this->_next_joint_publish)
    return;
  this->_next_joint_publish = std::max(this->_next_joint_publish + joint_period_us, now); // late: don't catch up
  float latest[NB_MOTORS];
  this->latest_positions(latest);
  for(int i = 0; i < NB_MOTORS; i++)
    joint_positions[i] = latest[i] * (M_PI / 4.0); // 1 = 45 deg
  int64_t ns = this->_time_synced ? rmw_uros_epoch_nanos() : now * 1000;
  joint_state.header.stamp.sec = (int32_t) (ns / 1000000000);
  joint_state.header.stamp.nanosec = (uint32_t) (ns % 1000000000);
  RCSOFTCHECK(rcl_publish(&joint_publisher, &joint_state, NULL));
}

// ----- Battery and motors ----- //

// Return the battery's voltage. Given that we use NIMH batteries, the voltage can exceed 5.14V, which is the maximum value that the battery monitor can report. When this happens, the monitor return unexpectedly low values (e.g., 0.01V, or 2.1V). Therefore, this function return 5.15V when the measured value is below 3.5V.
float Qutee::battery_voltage()
{
  if (!this->_battery_monitor_available)
    return 0.0f;
  float voltage = this->_maxlipo.cellVoltage();
  if (voltage<3.5)
    {
      voltage = 5.15;
      ESP_LOGW("BATTERY: ","measured voltage unexpectedly low. We assume that it means that the battery's voltage is higher than 5.14V. Returning 5.15V instead. ");
    }
  return voltage;
}

void Qutee::scan()
{
  int8_t found_dynamixel = 0;
  for(int id = 0; id < DXL_BROADCAST_ID; id++) {
      if(this->_dxl.ping(id)) {
      ESP_LOGI("DXL: ","ID : %i  , Model Number: %i",id,this->_dxl.getModelNumber(id));
      found_dynamixel++;
      }
  }
  ESP_LOGI("DXL: ","Total %i DYNAMIXEL(s) found!",found_dynamixel );
  char msg[32];
  snprintf(msg, sizeof(msg), "Motors found: %i/%i", found_dynamixel, NB_MOTORS);
  this->tft_status(msg, found_dynamixel == NB_MOTORS ? ST77XX_GREEN : ST77XX_YELLOW);
  delay(1000);
}

// With USB plugged in and the motor battery off, the Feather's charger feeds the motor bus through VBAT.
// The motors then start underpowered and latch an input voltage error, which only a reboot clears.
// So wait until the motors report a real supply voltage, then reboot any motor with a latched error.
void Qutee::check_motor_power()
{
  const float min_voltage = 4.6f; // the NiMH pack is about 5.2-5.5 V; USB back-feed gives less
  bool waiting_shown = false;
  int answered = 0;
  while(true){
    answered = 0;
    int lowest_id = -1;
    float lowest_voltage = 100.0f;
    for(int i = 0; i < NB_MOTORS; i++){
      int32_t raw = this->_dxl.readControlTableItem(PRESENT_INPUT_VOLTAGE, DXL_IDs[i]);
      if(this->_dxl.getLastLibErrCode() != DXL_LIB_OK)
        continue;
      answered++;
      if(raw / 10.0f < lowest_voltage){
        lowest_voltage = raw / 10.0f;
        lowest_id = DXL_IDs[i];
      }
    }
    // No motor answering is reported later by scan()
    if(answered == 0 || lowest_voltage >= min_voltage)
      break;
    if(!waiting_shown){
      ESP_LOGW("DXL: ","Motor %d reads %.1f V: waiting for the motor battery", lowest_id, lowest_voltage);
      this->tft_status("Switch on the motor battery", ST77XX_YELLOW);
      waiting_shown = true;
    }
    delay(500);
  }
  if(answered == 0)
    return;
  if(waiting_shown)
    delay(500); // let the supply settle after the battery is switched on

  this->reboot_faulted_motors(true);
}

// Hardware Error Status of a motor: 0 if none, -1 if it couldn't be read. It is one byte with only bits
// 0, 2, 3, 4 and 5 used, so anything else is a garbled reply (seen once as 0x9e0) and is reported as -1.
int32_t Qutee::read_hardware_error(int id)
{
  int32_t error = this->_dxl.readControlTableItem(HARDWARE_ERROR_STATUS, id);
  if(this->_dxl.getLastLibErrCode() != DXL_LIB_OK)
    return -1;
  if(error & ~0x3D){
    ESP_LOGW("DXL: ", "Motor %d: ignoring an invalid hardware error reading 0x%lx", id, (long) error);
    return -1;
  }
  return error;
}

// Reboot the motors with a latched hardware error (a fault turns the motor's torque off until it
// is rebooted), and add each fault to the motor report. Returns the number of motors rebooted.
// The caller must set up the rebooted motors again (init_motors), since a reboot resets them.
int Qutee::reboot_faulted_motors(bool log_healthy)
{
  int rebooted = 0;
  for(int i = 0; i < NB_MOTORS; i++){
    int id = DXL_IDs[i];
    int32_t error = this->read_hardware_error(id);
    if(error < 0)
      continue;
    float voltage = this->_dxl.readControlTableItem(PRESENT_INPUT_VOLTAGE, id) / 10.0f;
    if(error == 0){
      if(log_healthy)
        ESP_LOGI("DXL: ","Motor %d: %.1f V, no error", id, voltage);
      continue;
    }
    // Bits: 0 input voltage, 2 overheating, 3 motor encoder, 4 electrical shock, 5 overload
    std::string names = std::string((error & 0x01) ? " input-voltage" : "") + ((error & 0x04) ? " overheating" : "")
                      + ((error & 0x08) ? " encoder" : "") + ((error & 0x10) ? " electrical-shock" : "")
                      + ((error & 0x20) ? " overload" : "");
    ESP_LOGW("DXL: ","Motor %d: %.1f V, hardware error 0x%02lx%s, rebooting it", id, voltage, (long) error, names.c_str());
    this->_motor_report += "motor " + std::to_string(id) + names + " (rebooted); ";
    this->_dxl.reboot(id);
    rebooted++;
  }
  if(rebooted == 0)
    return 0;
  delay(1000); // the motors take a moment to restart

  for(int i = 0; i < NB_MOTORS; i++){
    int id = DXL_IDs[i];
    int32_t error = this->read_hardware_error(id);
    if(error > 0){
      ESP_LOGE("DXL: ","Motor %d still has hardware error 0x%02lx after a reboot", id, (long) error);
      this->_motor_report += "motor " + std::to_string(id) + " still faulty after reboot; ";
      char msg[40];
      snprintf(msg, sizeof(msg), "Motor %d error 0x%02lx", id, (long) error);
      this->tft_status(msg, ST77XX_RED);
      delay(2000);
    }
  }
  return rebooted;
}

void Qutee::init_motors()
{
  for(size_t i = 0; i < NB_MOTORS; i++)
  {
    // Turn off torque when configuring items in EEPROM area
    this->_dxl.torqueOff(this->DXL_IDs[i]);
    this->_dxl.setOperatingMode(this->DXL_IDs[i], OP_POSITION);
    // Time-based motion profile (Drive Mode bit 2, an EEPROM item, so with the torque off): Profile Velocity
    // is then the time to reach a target (see set_servo_motion). Keep the other bits (bit 0 reverses the motor).
    int32_t drive_mode = this->_dxl.readControlTableItem(DRIVE_MODE, DXL_IDs[i]);
    if(this->_dxl.getLastLibErrCode() == DXL_LIB_OK && !(drive_mode & 0x04))
      this->_dxl.writeControlTableItem(DRIVE_MODE, DXL_IDs[i], drive_mode | 0x04);
    // Full output (a controller may lower it; it stays in EEPROM if the robot was switched off meanwhile)
    int32_t pwm_limit = this->_dxl.readControlTableItem(PWM_LIMIT, DXL_IDs[i]);
    if(this->_dxl.getLastLibErrCode() == DXL_LIB_OK && pwm_limit != 885)
      this->_dxl.writeControlTableItem(PWM_LIMIT, DXL_IDs[i], 885);
    this->_dxl.torqueOn(this->DXL_IDs[i]);
    // No motion profile (0): the motor reaches each target as fast as it can
    this->_dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_IDs[i], 0);
    this->_dxl.writeControlTableItem(PROFILE_ACCELERATION, DXL_IDs[i], 0);
  }
  this->set_PID_gains();
  ESP_LOGI("DXL: ","Torque on");
  this->_torque_on = true;
}

void Qutee::set_PID_gains()
{
  // adjust these values depending on how your robot behaves. 0.05 looks like a good target to have on all motors during checkup.
  uint16_t position_p_gain = 1600;
  uint16_t position_i_gain = 0;
  uint16_t position_d_gain = 40;
  for(size_t i = 0; i < NB_MOTORS; i++)
  {
    this->_dxl.writeControlTableItem(POSITION_P_GAIN, this->DXL_IDs[i], position_p_gain);
    this->_dxl.writeControlTableItem(POSITION_I_GAIN, this->DXL_IDs[i], position_i_gain);
    this->_dxl.writeControlTableItem(POSITION_D_GAIN, this->DXL_IDs[i], position_d_gain);
  }
}

// Pose with joint 2 of every leg raised by angle_deg (positive joint 2 moves a leg down) and the
// other joints at neutral. 0 is the standing pose; on the ground, raising the legs lowers the torso.
Qutee::Actions_t Qutee::legs_raised_pose(float angle_deg)
{
  Actions_t pose;
  pose.setZero();
  for(size_t leg = 0; leg < 4; leg++)
    pose(leg * 3 + 1, 0) = -angle_deg / 45.0f; // actions: 1 = 45 deg
  return pose;
}

// Move from the current joint positions to target, easing in and out, instead of jumping there
void Qutee::move_slowly(const Actions_t& target, int duration_ms)
{
  if(!this->_torque_on)
    this->set_torque(true);
  State_t state;
  this->get_motor_positions(state, 6);
  Actions_t start;
  for(size_t i = 0; i < NB_MOTORS; i++)
    start(i, 0) = state(6 + i, 0);
  int steps = std::max(1, duration_ms * CONFIG_CONTROL_FREQUENCY / 1000);
  for(int s = 1; s <= steps; s++){
    float progress = 0.5f - 0.5f * cosf(M_PI * s / steps); // 0 to 1, slow at both ends
    Actions_t command = start + (target - start) * progress;
    this->send_actions(command);
    delay(1000 / CONFIG_CONTROL_FREQUENCY);
  }
  // Report where joint 2 of each leg ended up: it falls short of the target when the legs are loaded
  delay(200);
  this->get_motor_positions(state, 6);
  ESP_LOGI("Pose", "joint 2 target %.0f deg, reached: leg1 %.0f, leg2 %.0f, leg3 %.0f, leg4 %.0f",
           target(1, 0) * 45.0f, state(6 + 1, 0) * 45.0f, state(6 + 4, 0) * 45.0f,
           state(6 + 7, 0) * 45.0f, state(6 + 10, 0) * 45.0f);
}

// With the time-based profile (set in init_motors), Profile Velocity is the time to reach each new target and
// Profile Acceleration the time spent accelerating; 0 means as fast as possible. Both are RAM: they can change
// while the torque is on.
void Qutee::set_servo_motion(int profile_ms, int p_gain)
{
  for(size_t i = 0; i < NB_MOTORS; i++){
    this->_dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_IDs[i], profile_ms);
    this->_dxl.writeControlTableItem(PROFILE_ACCELERATION, DXL_IDs[i], profile_ms / 4);
    this->_dxl.writeControlTableItem(POSITION_P_GAIN, DXL_IDs[i], p_gain);
  }
  ESP_LOGI("DXL: ", "Servo profile %d ms, P gain %d", profile_ms, p_gain);
}

// PWM Limit is an EEPROM item: the caller must have turned the torque off. Only motors whose value
// differs are written, to spare their EEPROM.
void Qutee::set_pwm_limit(int limit)
{
  for(size_t i = 0; i < NB_MOTORS; i++){
    int32_t current = this->_dxl.readControlTableItem(PWM_LIMIT, DXL_IDs[i]);
    if(this->_dxl.getLastLibErrCode() == DXL_LIB_OK && current != limit)
      this->_dxl.writeControlTableItem(PWM_LIMIT, DXL_IDs[i], limit);
  }
  ESP_LOGI("DXL: ", "PWM limit %d (%.0f%%)", limit, limit / 8.85f);
}

// Bring back one motor after a fault (e.g. overload) while the others keep running: reboot it, let it cool
// down, and set it up again without a jump (goal = where the joint is now). PWM Limit survives a reboot.
void Qutee::recover_motor(int id, int profile_ms, int p_gain)
{
  this->_dxl.reboot(id);
  delay(2000);
  this->_dxl.torqueOff(id);
  this->_dxl.setOperatingMode(id, OP_POSITION);
  int32_t drive_mode = this->_dxl.readControlTableItem(DRIVE_MODE, id);
  if(this->_dxl.getLastLibErrCode() == DXL_LIB_OK && !(drive_mode & 0x04))
    this->_dxl.writeControlTableItem(DRIVE_MODE, id, drive_mode | 0x04);
  int32_t position = this->_dxl.readControlTableItem(PRESENT_POSITION, id);
  if(this->_dxl.getLastLibErrCode() == DXL_LIB_OK)
    this->_dxl.writeControlTableItem(GOAL_POSITION, id, position);
  this->_dxl.torqueOn(id);
  this->_dxl.writeControlTableItem(PROFILE_VELOCITY, id, profile_ms);
  this->_dxl.writeControlTableItem(PROFILE_ACCELERATION, id, profile_ms / 4);
  this->_dxl.writeControlTableItem(POSITION_P_GAIN, id, p_gain);
  this->_dxl.writeControlTableItem(POSITION_I_GAIN, id, 0);
  this->_dxl.writeControlTableItem(POSITION_D_GAIN, id, 40);
}

void Qutee::go_to_rest_pose()
{
  this->move_slowly(this->legs_raised_pose(CONFIG_LEG_REST_ANGLE), CONFIG_POSE_TRANSITION_TIME_MS);
  if(this->_torque_off_at_rest)
    this->set_torque(false); // the torso settles on the ground; move_slowly turns the torque back on
}

void Qutee::set_torque(bool on)
{
  if(on){
    // A motor drives to its stored goal when its torque comes on: set the goal to where the joint
    // is now, so the legs don't jump if they moved while limp
    State_t state;
    this->get_motor_positions(state, 6);
    Actions_t here;
    for(size_t i = 0; i < NB_MOTORS; i++)
      here(i, 0) = state(6 + i, 0);
    this->send_actions(here);
  }
  // A motor can miss the command (its reply lost on the bus), so read Torque Enable back and retry
  for(size_t i = 0; i < NB_MOTORS; i++){
    int id = DXL_IDs[i];
    bool done = false;
    int attempt;
    for(attempt = 1; attempt <= 3 && !done; attempt++){
      if(on)
        this->_dxl.torqueOn(id);
      else
        this->_dxl.torqueOff(id);
      int32_t enabled = this->_dxl.readControlTableItem(TORQUE_ENABLE, id);
      done = this->_dxl.getLastLibErrCode() == DXL_LIB_OK && enabled == (on ? 1 : 0);
    }
    if(!done){
      ESP_LOGW("DXL: ","Motor %d: torque did not turn %s", id, on ? "on" : "off");
      this->_motor_report += "motor " + std::to_string(id) + " torque did not turn " + (on ? "on" : "off") + "; ";
    } else if(attempt > 2){
      ESP_LOGW("DXL: ","Motor %d: torque turned %s after %d attempts", id, on ? "on" : "off", attempt - 1);
    }
  }
  this->_torque_on = on;
  ESP_LOGI("DXL: ","Torque %s", on ? "on" : "off (rest)");
}

// Stand up through the "legs up" pose, like at boot: at rest (especially with the torque off) the
// legs drift, hips splayed and knees folded, and can't lift the body from there. Raised, they carry
// no load and realign; lowered from there, they lift the body.
void Qutee::go_to_standing_pose()
{
  this->move_slowly(this->legs_raised_pose(CONFIG_LEG_UP_ANGLE), CONFIG_POSE_TRANSITION_TIME_MS);
  this->move_slowly(this->legs_raised_pose(0), CONFIG_POSE_TRANSITION_TIME_MS);
}

void Qutee::displaySensorStatus(void)
{
  uint8_t system_status, self_test_results, system_error;
  system_status = self_test_results = system_error = 0;
  this->_bno.getSystemStatus(&system_status, &self_test_results, &system_error);
  ESP_LOGI("IMU" ,"System Status: 0x%x, Self Test: 0x%x, System Error:  0x%x", system_status, self_test_results, system_error);
  delay(500);
}

void Qutee::get_state(State_t& state_ref)
{
  sensors_event_t orientationData = {}, linearAccelData = {};
  if(this->_imu_available){
    this->_bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    this->_bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
  }

  state_ref.setValues({{this->DEG_2_RAD * orientationData.orientation.x},
                  {this->DEG_2_RAD * orientationData.orientation.y},
                  {this->DEG_2_RAD * orientationData.orientation.z},
                  {linearAccelData.acceleration.x},
                  {linearAccelData.acceleration.y},
                  {linearAccelData.acceleration.z}});

  this->get_motor_positions(state_ref,6);
}

void Qutee::send_actions(const Actions_t& actions){
  for(uint8_t i = 0; i < NB_MOTORS; i++){
    this->_sw_data[i].goal_position = (size_t) 2048+actions(i,0)*512;
  }
  this->_sw_infos.is_info_changed = true;
  // Build a SyncWrite Packet and transmit to DYNAMIXEL
  this->_dxl.syncWrite(&this->_sw_infos);
}

void Qutee::get_motor_positions(State_t& state_ref, size_t offset)
{
  // SyncRead stops at the first motor that doesn't answer, so only the first recv_cnt motors
  // are fresh. The others keep their last known position.
  // Replies get lost now and then, more often while the legs carry the robot (higher currents).
  // A SyncRead takes about 4 ms of the 20 ms control period, so retry once.
  uint8_t i, recv_cnt;
  recv_cnt = this->_dxl.syncRead(&this->_sr_infos);
  if(recv_cnt < NB_MOTORS)
    recv_cnt = this->_dxl.syncRead(&this->_sr_infos);
  for(i = 0; i<recv_cnt; i++)
    this->_last_positions[i] = (this->_sr_data[i].present_position-2048.0f)/512.0f;
  if(recv_cnt < NB_MOTORS)
    this->_incomplete_reads++;
  for(i = 0; i<NB_MOTORS; i++)
    state_ref(i+offset,0) = this->_last_positions[i];
  {
    std::lock_guard<std::mutex> lock(this->_published_positions_lock);
    std::copy(this->_last_positions, this->_last_positions + NB_MOTORS, this->_published_positions);
    this->_published_positions_time = esp_timer_get_time();
  }
  // A read on the ROS task happens during a service callback (e.g. a rollout): publish from here
  if(this->_joint_publisher_ready && xTaskGetCurrentTaskHandle() == this->_ros_task_handle)
    this->publish_joint_states_if_due();
}

// The newest joint positions: those read by the activity that has the motors, or, when the motors are
// idle, read now (taking the motors for the few ms of a SyncRead)
void Qutee::latest_positions(float* positions){
  const char* expected = nullptr;
  bool idle = false;
  {
    std::lock_guard<std::mutex> lock(this->_published_positions_lock);
    idle = esp_timer_get_time() - this->_published_positions_time > 30000; // no read in the last 30 ms
  }
  if(idle && this->_motors_owner.compare_exchange_strong(expected, JOINT_READING)){
    State_t state;
    this->get_motor_positions(state, 6);
    this->release_motors();
  }
  std::lock_guard<std::mutex> lock(this->_published_positions_lock);
  std::copy(this->_published_positions, this->_published_positions + NB_MOTORS, positions);
}

// ----- Checkup ----- //

// Stands up, then moves all motors in a slow sine wave and shows each one's average position error,
// until a button is pressed. The caller holds the motors.
void Qutee::checkup(){
  {
    std::lock_guard<std::recursive_mutex> lock(this->_tft_lock);
    this->_tft.fillRect(0, 0, 240, MENU_HEIGHT, ST77XX_BLACK);
    this->_tft.setTextSize(2);
    this->_tft.setCursor(0, 0);
    this->_tft.setTextColor(ST77XX_BLUE);
    this->_tft.println("CHECKUP");
  }
  this->tft_status("Standing up...");
  wait_button_release();

  go_to_standing_pose();
  delay(1000);
  int64_t start, sleep_duration;
  int64_t start_loop = esp_timer_get_time();
  State_t state;
  Actions_t actions;
  float action_value=0;
  int32_t step = 0;
  float er[NB_MOTORS] = {};

  this->tft_init_checkup_screen();
  this->tft_status("Press button to exit");
  while(this->poll_button() == BUTTON_NONE)
  {
    start = esp_timer_get_time();
    this->get_motor_positions(state,6);

    if(step % (2 * CONFIG_CONTROL_FREQUENCY)==0)
      this->tft_update_checkup_screen(er);

    for(size_t i=0;i<NB_MOTORS;i++)
      er[i]= (step * er[i] + fabsf(state(6+i,0)-action_value))/(step+1.0f); //compute rolling average of error

    step++;

    action_value = sin(0.5*6.28*(esp_timer_get_time() - start_loop)/1000000.0f);
    actions.setConstant(action_value);
    this->send_actions(actions); // a sine wave at 0.5 Hz

    sleep_duration = (PERIOD - (esp_timer_get_time() - start));
    if(sleep_duration>0)
      std::this_thread::sleep_for(std::chrono::microseconds(sleep_duration));
    else
      ESP_LOGE("Checkup"," Error: loop duration: %" PRId64" longer than period: ", esp_timer_get_time() - start);
  }
  ESP_LOGI("Checkup", "duration of the loop: %" PRId64, esp_timer_get_time() - start_loop);
  this->tft_status("Going back to rest...");
  go_to_rest_pose();
  this->tft_status("Checkup done");
}

// Checkup screen layout (text size 2, 12x16 px characters): one row per leg, one column per joint,
// i.e. motor ID = 10*leg + joint
#define CHECKUP_LINE_OFFSET 22 // pixels between rows
#define CHECKUP_COL_X(joint) ((3 + 5 * (joint)) * 12) // x of the value column for joint 0..2

void Qutee::tft_init_checkup_screen() {
  std::lock_guard<std::recursive_mutex> lock(this->_tft_lock);
  this->_tft.fillRect(0, 0, 240, MENU_HEIGHT, ST77XX_BLACK);
  this->_tft.setTextSize(2);
  this->_tft.setTextColor(ST77XX_RED,ST77XX_BLACK);
  this->_tft.setCursor(0, 0);    this->_tft.print("Err");
  this->_tft.setTextColor(ST77XX_BLUE,ST77XX_BLACK);
  for(size_t joint = 0; joint < 3; joint++){
    this->_tft.setCursor(CHECKUP_COL_X(joint), 0); this->_tft.printf("J%d", joint + 1);
  }
  for(size_t leg = 0; leg < 4; leg++){
    this->_tft.setCursor(0, (leg + 1) * CHECKUP_LINE_OFFSET); this->_tft.printf("L%d", leg + 1);
  }
}

void Qutee::tft_update_checkup_screen(const float* err) {
  std::lock_guard<std::recursive_mutex> lock(this->_tft_lock);
  char buf[8];
  this->_tft.setTextSize(2);
  for(size_t i=0; i < NB_MOTORS; i++){
    if(err[i]>0.1)
      this->_tft.setTextColor(ST77XX_RED,ST77XX_BLACK);
    else
      this->_tft.setTextColor(ST77XX_WHITE,ST77XX_BLACK);
    snprintf(buf, sizeof(buf), "%4.2f", err[i]);
    this->_tft.setCursor(CHECKUP_COL_X(i%3), (i/3 + 1) * CHECKUP_LINE_OFFSET); this->_tft.print(buf);
  }
}
