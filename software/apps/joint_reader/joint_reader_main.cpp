// Joint reader application (build with QUTEE_APP=joint_reader: ./build.sh joint_reader).
// At boot: motors with the torque off, WiFi, then ROS (retrying until the agent answers). Publishes
//   /<robot name>/joint_states   sensor_msgs/JointState, 20 Hz: names "11".."43", positions in radians
//                                (0 = the standing pose)
// and offers the parameter
//   /<robot name>/joint_reader torque   true: the motors hold the current pose; false: move the legs by hand
// Logs also go over WiFi (tools/wifi_monitor.py), as in the full firmware.
#include <atomic>
#include <cmath>
#include <mutex>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_timer.h"

#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rclc_parameter/rclc_parameter.h>
#include <rmw_microros/rmw_microros.h>
#include <sensor_msgs/msg/joint_state.h>

#include "JointReader.hpp"
#include "udp_log.hpp"

static const char* TAG = "JointReader";
#define RCCHECK(fn) { rcl_ret_t rc = fn; if(rc != RCL_RET_OK){ESP_LOGE(TAG, "Failed status on line %d: %d", __LINE__, (int) rc); vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t rc = fn; if(rc != RCL_RET_OK){ESP_LOGW(TAG, "Failed status on line %d: %d", __LINE__, (int) rc);}}

static JointReader reader;
static std::mutex latest_lock;
static JointReader::Angles latest = {};
static int latest_answered = 0;
static std::atomic<int> torque_request{-1}; // -1 none, 0 off, 1 on (set from ROS, applied by the reading loop)

// ----- ROS -----
static rcl_publisher_t publisher;
static sensor_msgs__msg__JointState joint_state;
static char joint_names[JointReader::N][4];
static rosidl_runtime_c__String names[JointReader::N];
static double positions[JointReader::N];
static char empty_frame[] = "";
static rclc_parameter_server_t param_server;

static bool on_parameter_changed(const Parameter* old_param, const Parameter* new_param, void* context)
{
  if(new_param == NULL)
    return false;
  if(strcmp(new_param->name.data, "torque") == 0){
    torque_request = new_param->value.bool_value ? 1 : 0;
    return true;
  }
  return false;
}

static void init_joint_state()
{
  for(int i = 0; i < JointReader::N; i++){
    snprintf(joint_names[i], sizeof(joint_names[i]), "%d", JointReader::IDs[i]);
    names[i].data = joint_names[i];
    names[i].size = strlen(joint_names[i]);
    names[i].capacity = sizeof(joint_names[i]);
  }
  joint_state.name.data = names;
  joint_state.name.size = joint_state.name.capacity = JointReader::N;
  joint_state.position.data = positions;
  joint_state.position.size = joint_state.position.capacity = JointReader::N;
  joint_state.velocity.size = joint_state.effort.size = 0;
  joint_state.header.frame_id.data = empty_frame;
  joint_state.header.frame_id.size = 0;
  joint_state.header.frame_id.capacity = sizeof(empty_frame);
}

static void ros_task(void* arg)
{
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  RCCHECK(rcl_init_options_init(&init_options, allocator));
  rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
  RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));
  // A client key that stays the same across resets, but differs from the full firmware's: the agent keeps a
  // session's entities, so sharing the key would mix the two firmwares' services up
  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_WIFI_STA);
  uint32_t client_key = (((uint32_t) mac[2] << 24) | ((uint32_t) mac[3] << 16) | ((uint32_t) mac[4] << 8) | mac[5])
                        ^ 0x4A520000;  // "JR"
  RCCHECK(rmw_uros_options_set_client_key(client_key, rmw_options));

  rclc_support_t support;
  reader.status("Connecting to agent " CONFIG_MICRO_ROS_AGENT_IP);
  while(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator) != RCL_RET_OK){
    reader.status("Agent unreachable, retrying...", ST77XX_RED);
    vTaskDelay(pdMS_TO_TICKS(2000));
  }

  rcl_node_t node;
  RCCHECK(rclc_node_init_default(&node, "joint_reader", reader.name().c_str(), &support));
  init_joint_state();
  RCCHECK(rclc_publisher_init_best_effort(&publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
                                          "joint_states"));

  rclc_parameter_options_t options = {};
  options.notify_changed_over_dds = false;
  options.max_params = 2;
  options.allow_undeclared_parameters = false;
  options.low_mem_mode = false;
  RCCHECK(rclc_parameter_server_init_with_option(&param_server, &node, &options));
  RCCHECK(rclc_add_parameter(&param_server, "torque", RCLC_PARAMETER_BOOL));
  RCCHECK(rclc_parameter_set_bool(&param_server, "torque", false));

  rclc_executor_t executor;
  RCCHECK(rclc_executor_init(&executor, &support.context, RCLC_EXECUTOR_PARAMETER_SERVER_HANDLES, &allocator));
  RCCHECK(rclc_executor_add_parameter_server(&executor, &param_server, on_parameter_changed));

  // Time stamps from the agent's clock when available
  bool time_synced = rmw_uros_sync_session(1000) == RMW_RET_OK;
  char msg[48];
  snprintf(msg, sizeof(msg), "ROS ready: /%s/joint_states", reader.name().c_str());
  reader.status(msg, ST77XX_GREEN);
  ESP_LOGI(TAG, "%s (time %s)", msg, time_synced ? "synced with the agent" : "since boot");

  // Publishing at 20 Hz also keeps the UDP flow to the agent alive (Docker's port forwarding drops idle flows)
  const int64_t period_us = 50000;
  int64_t next = esp_timer_get_time();
  while(true){
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5)));
    if(esp_timer_get_time() < next){
      vTaskDelay(pdMS_TO_TICKS(5));
      continue;
    }
    next += period_us;
    {
      std::lock_guard<std::mutex> lock(latest_lock);
      for(int i = 0; i < JointReader::N; i++)
        positions[i] = latest[i] * M_PI / 180.0;
    }
    int64_t ns = time_synced ? rmw_uros_epoch_nanos() : esp_timer_get_time() * 1000;
    joint_state.header.stamp.sec = (int32_t) (ns / 1000000000);
    joint_state.header.stamp.nanosec = (uint32_t) (ns % 1000000000);
    RCSOFTCHECK(rcl_publish(&publisher, &joint_state, NULL));
  }
}

// ----- WiFi -----
static void start_wifi()
{
  reader.status("Connecting to WiFi " CONFIG_ESP_WIFI_SSID);
  ESP_ERROR_CHECK(uros_network_interface_initialize());
  esp_netif_ip_info_t ip_info = {};
  esp_netif_t* netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
  if(netif && esp_netif_get_ip_info(netif, &ip_info) == ESP_OK && ip_info.ip.addr != 0){
    char msg[40];
    snprintf(msg, sizeof(msg), "WiFi connected, IP " IPSTR, IP2STR(&ip_info.ip));
    reader.status(msg, ST77XX_GREEN);
    udp_log_start();
  } else {
    reader.status("WiFi failed: check SSID/password", ST77XX_RED);
  }
}

extern "C" void app_main()
{
  udp_log_init();
  reader.init();
  start_wifi();
  xTaskCreate(ros_task, "uros_task", CONFIG_MICRO_ROS_APP_STACK, NULL, CONFIG_MICRO_ROS_APP_TASK_PRIO, NULL);

  // Read the joints at 20 Hz; the screen shows them 4 times a second
  JointReader::Angles angles;
  int step = 0;
  while(true){
    int request = torque_request.exchange(-1);
    if(request != -1 && (request == 1) != reader.torque()){
      reader.set_torque(request == 1);
      reader.status(request == 1 ? "Torque on: holding" : "Torque off: move the legs by hand",
                    request == 1 ? ST77XX_YELLOW : ST77XX_GREEN);
    }
    int answered = reader.read_positions(angles);
    {
      std::lock_guard<std::mutex> lock(latest_lock);
      latest = angles;
      latest_answered = answered;
    }
    if(step++ % 5 == 0)
      reader.show(angles, answered);
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}
