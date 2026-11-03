#include "JointReader.hpp"

#include <sstream>
#include <vector>

#include "driver/ledc.h"
#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"

using namespace ControlTableItem;

const int JointReader::IDs[JointReader::N] = {11, 12, 13, 21, 22, 23, 31, 32, 33, 41, 42, 43};

static const char* TAG = "JointReader";

JointReader::JointReader() : _port(UART_NUM_1), _tft(TFT_CS, TFT_DC, TFT_RST) {}

void JointReader::init()
{
  this->load_name();
  this->init_screen();
  this->status("Starting motors...");

  // Motors: same bus settings as the full firmware (3 Mbps, protocol 2.0)
  this->_dxl.setPort(this->_port);
  this->_dxl.setPortProtocolVersion(2.0f);
  this->_dxl.begin(3000000);

  this->_sr.packet.p_buf = this->_sr_buf;
  this->_sr.packet.buf_capacity = sizeof(this->_sr_buf);
  this->_sr.packet.is_completed = false;
  this->_sr.addr = 132;       // Present Position
  this->_sr.addr_length = 4;
  this->_sr.p_xels = this->_sr_xels;
  this->_sr.xel_count = N;
  for(int i = 0; i < N; i++){
    this->_sr_xels[i].id = IDs[i];
    this->_sr_xels[i].p_recv_buf = (uint8_t*) &this->_positions[i];
  }
  this->_sr.is_info_changed = true;

  int found = 0;
  for(int i = 0; i < N; i++)
    if(this->_dxl.ping(IDs[i]))
      found++;
  ESP_LOGI(TAG, "%d of %d motors found", found, N);
  this->set_torque(false);

  char msg[40];
  snprintf(msg, sizeof(msg), "Motors found: %d/%d, torque off", found, N);
  this->status(msg, found == N ? ST77XX_GREEN : ST77XX_YELLOW);
}

int JointReader::read_positions(Angles& degrees)
{
  int answered = this->_dxl.syncRead(&this->_sr);
  for(int i = 0; i < answered; i++)
    this->_last[i] = (this->_positions[i].value - 2048) * (360.0f / 4096.0f);
  degrees = this->_last;
  return answered;
}

void JointReader::set_torque(bool on)
{
  for(int i = 0; i < N; i++){
    if(on){
      // Hold where the joint is now: a motor drives to its stored goal when its torque comes on
      int32_t position = this->_dxl.readControlTableItem(PRESENT_POSITION, IDs[i]);
      if(this->_dxl.getLastLibErrCode() == DXL_LIB_OK)
        this->_dxl.writeControlTableItem(GOAL_POSITION, IDs[i], position);
      this->_dxl.torqueOn(IDs[i]);
    } else {
      this->_dxl.torqueOff(IDs[i]);
    }
  }
  this->_torque = on;
  ESP_LOGI(TAG, "Torque %s", on ? "on (holding)" : "off (move the legs by hand)");
}

// ----- screen -----

void JointReader::init_screen()
{
  // Backlight through LEDC at half brightness, as in the full firmware
  ledc_timer_config_t timer = {};
  timer.speed_mode = LEDC_LOW_SPEED_MODE;
  timer.duty_resolution = LEDC_TIMER_13_BIT;
  timer.timer_num = LEDC_TIMER_0;
  timer.freq_hz = 4000;
  timer.clk_cfg = LEDC_AUTO_CLK;
  ESP_ERROR_CHECK(ledc_timer_config(&timer));
  ledc_channel_config_t channel = {};
  channel.speed_mode = LEDC_LOW_SPEED_MODE;
  channel.channel = LEDC_CHANNEL_0;
  channel.timer_sel = LEDC_TIMER_0;
  channel.intr_type = LEDC_INTR_DISABLE;
  channel.gpio_num = (gpio_num_t) TFT_BACKLITE;
  channel.duty = 4096;
  ESP_ERROR_CHECK(ledc_channel_config(&channel));
  gpio_set_direction((gpio_num_t) TFT_I2C_POWER, GPIO_MODE_OUTPUT);
  gpio_set_level((gpio_num_t) TFT_I2C_POWER, 1); // screen (and I2C) power
  delay(10);
  this->_tft.init(135, 240);
  this->_tft.setRotation(3);
  this->_tft.fillScreen(ST77XX_BLACK);
  this->_tft.setTextWrap(false);
  this->_tft.setTextSize(2);
  this->_tft.setTextColor(ST77XX_CYAN, ST77XX_BLACK);
  this->_tft.setCursor(0, 0);
  this->_tft.print("Joint reader ");
  this->_tft.print(this->_name.c_str());
}

void JointReader::show(const Angles& degrees, int answered)
{
  // One row per leg, one column per joint: "L1  -12.3  45.0   3.2"
  this->_tft.setTextSize(2);
  for(int leg = 0; leg < 4; leg++){
    this->_tft.setCursor(0, 22 + leg * 22);
    this->_tft.setTextColor(ST77XX_YELLOW, ST77XX_BLACK);
    this->_tft.printf("L%d", leg + 1);
    this->_tft.setTextColor(answered == N ? ST77XX_WHITE : ST77XX_RED, ST77XX_BLACK);
    for(int joint = 0; joint < 3; joint++){
      char buf[8];
      snprintf(buf, sizeof(buf), "%6.1f", degrees[leg * 3 + joint]);
      this->_tft.setCursor(30 + joint * 70, 22 + leg * 22);
      this->_tft.print(buf);
    }
  }
}

void JointReader::status(const char* msg, uint16_t color)
{
  this->_tft.fillRect(0, 128, 240, 8, ST77XX_BLACK);
  this->_tft.setTextSize(1);
  this->_tft.setTextColor(color, ST77XX_BLACK);
  this->_tft.setCursor(0, 128);
  this->_tft.print(msg);
}

// The robot's name is its ROS namespace: the one chosen in the full firmware's menu (stored in NVS),
// otherwise the first of the names in menuconfig (QUTEE settings).
void JointReader::load_name()
{
  std::vector<std::string> names;
  std::stringstream ss(CONFIG_QUTEE_NAME);
  std::string n;
  while(std::getline(ss, n, ';'))
    names.push_back(n);
  size_t id = 0;
  if(nvs_flash_init() == ESP_OK){
    nvs_handle_t handle;
    if(nvs_open("storage", NVS_READONLY, &handle) == ESP_OK){
      size_t size = sizeof(id);
      nvs_get_blob(handle, "name_id", &id, &size);
      nvs_close(handle);
    }
  }
  this->_name = id < names.size() ? names[id] : (names.empty() ? "qutee" : names[0]);
}
