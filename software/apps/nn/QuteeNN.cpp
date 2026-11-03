#include "QuteeNN.hpp"

#include <inttypes.h>
#include <thread>
#include "esp_log.h"
#include "esp_timer.h"

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ESP_LOGE("UROS","Failed status on line %d: %d. Aborting.",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}

static char time_label[5] = "time";
static char action_label[7] = "action";
static char state_label[6] = "state";
static char weights_label[8] = "weights";

// The client key salt 0 keeps the key of the original firmware, whose services this one offers
QuteeNN::QuteeNN():
  Qutee("Neural network", 0),
  _states(NB_STEPS),
  _actions(NB_STEPS)
{
}

// ----- Episode ----- //

void QuteeNN::control_step(State_t& state_to_fill, Actions_t& actions_to_fill){
  this->get_state(state_to_fill);
  this->_policy.pos(state_to_fill, actions_to_fill);
  this->send_actions(actions_to_fill);
}

void QuteeNN::run_episode(){
  // A motor that tripped (e.g. overload) in an earlier episode stays limp until it is rebooted
  this->set_motor_report("");
  if(this->reboot_faulted_motors(false) > 0)
    this->init_motors(); // a reboot turns torque off and resets the gains
  this->tft_status("Rollout: standing up", ST77XX_YELLOW);
  this->go_to_standing_pose();
  std::this_thread::sleep_for(std::chrono::microseconds(1000000));
  this->tft_status("Rollout: running", ST77XX_YELLOW);
  this->reset_incomplete_reads(); // count only the episode's reads
  int64_t start, sleep_duration;
  int64_t start_loop = esp_timer_get_time();
  for(size_t step = 0; step < NB_STEPS; step++)
  {
    State_t state_to_fill;
    Actions_t actions_to_fill;

    start = esp_timer_get_time();

    control_step(state_to_fill, actions_to_fill);
    _states[step] = state_to_fill;
    _actions[step] = actions_to_fill;

    sleep_duration = PERIOD - (esp_timer_get_time() - start);
    if(sleep_duration>0)
      std::this_thread::sleep_for(std::chrono::microseconds(sleep_duration));
    else
      ESP_LOGE("Control Loop"," Error: loop duration: %" PRId64" longer than period: ", esp_timer_get_time() - start);
  }
  ESP_LOGI("Control Loop", "duration of the loop: %" PRId64, esp_timer_get_time() - start_loop);
  if(this->incomplete_reads() > 0){
    ESP_LOGW("Control Loop", "%d of %d motor reads were incomplete; missing positions kept their last value",
             this->incomplete_reads(), (int) NB_STEPS);
    this->add_to_motor_report(std::to_string(this->incomplete_reads()) + " incomplete motor reads; ");
  }
  this->tft_status("Rollout: back to rest", ST77XX_YELLOW);
  this->go_to_rest_pose();
  this->tft_status("Rollout done", ST77XX_GREEN);
}

// ----- ROS: the rollout service ----- //

void QuteeNN::ros_setup(rcl_node_t* node, rclc_executor_t* executor, rclc_parameter_server_t* params){
  (void) params;
  this->init_rollout_messages();
  RCCHECK(rclc_service_init_default(&_service_rollout, node, ROSIDL_GET_SRV_TYPE_SUPPORT(qutee_interface, srv, Rollout), "rollout"));
  RCCHECK(rclc_executor_add_service(executor, &_service_rollout, &_req_rollout, &_res_rollout, rollout_callback));
}

// The messages' buffers are allocated once, at their final size.
// WARNING: a response is limited to 32768 bytes; with headers, about 32000 bytes are left for the
// NB_STEPS * (NN_OUTPUT_SIZE + NN_INPUT_SIZE) * 4 bytes of states and actions.
static void init_multi_array(std_msgs__msg__Float32MultiArray& array, size_t nb_steps, size_t size, char* label, size_t label_size){
  array.layout.dim.size = 2;
  array.layout.dim.capacity = 2;
  array.layout.dim.data = (std_msgs__msg__MultiArrayDimension*) malloc(2 * sizeof(std_msgs__msg__MultiArrayDimension));
  array.layout.dim.data[0].label.capacity = 5;
  array.layout.dim.data[0].label.size = 5;
  array.layout.dim.data[0].label.data = time_label;
  array.layout.dim.data[0].size = nb_steps;
  array.layout.dim.data[0].stride = nb_steps * size;
  array.layout.dim.data[1].label.capacity = label_size;
  array.layout.dim.data[1].label.size = label_size;
  array.layout.dim.data[1].label.data = label;
  array.layout.dim.data[1].size = size;
  array.layout.dim.data[1].stride = size;
  array.layout.data_offset = 0;
  array.data.capacity = nb_steps * size;
  array.data.size = nb_steps * size;
  array.data.data = (float*) malloc(array.data.capacity * sizeof(float));
}

void QuteeNN::init_rollout_messages(){
  size_t nb_weights = this->_policy.get_number_weights();
  ESP_LOGI("ROS: ","Init message: step %d, action %d, state %d, nb_weights %d", (int) NB_STEPS, NN_OUTPUT_SIZE, NN_INPUT_SIZE, (int) nb_weights);
  // The structure of the response is: time x action (or state) dimension
  init_multi_array(_res_rollout.actions, NB_STEPS, NN_OUTPUT_SIZE, action_label, 7);
  init_multi_array(_res_rollout.states, NB_STEPS, NN_INPUT_SIZE, state_label, 6);

  _req_rollout.weights.layout.dim.size = 1;
  _req_rollout.weights.layout.dim.capacity = 1;
  _req_rollout.weights.layout.dim.data = (std_msgs__msg__MultiArrayDimension*) malloc(sizeof(std_msgs__msg__MultiArrayDimension));
  _req_rollout.weights.layout.dim.data[0].label.capacity = 8;
  _req_rollout.weights.layout.dim.data[0].label.size = 8;
  _req_rollout.weights.layout.dim.data[0].label.data = weights_label;
  _req_rollout.weights.layout.dim.data[0].size = nb_weights;
  _req_rollout.weights.layout.dim.data[0].stride = nb_weights;
  _req_rollout.weights.layout.data_offset = 0;
  _req_rollout.weights.data.capacity = nb_weights;
  _req_rollout.weights.data.size = nb_weights;
  _req_rollout.weights.data.data = (float*) malloc(nb_weights * sizeof(float));
}

void QuteeNN::fill_rollout_response(qutee_interface__srv__Rollout_Response* response){
  for(size_t i = 0; i < _actions.size(); i++)
    std::copy(_actions[i].data(), _actions[i].data() + NN_OUTPUT_SIZE, response->actions.data.data + i * NN_OUTPUT_SIZE);
  for(size_t i = 0; i < _states.size(); i++)
    std::copy(_states[i].data(), _states[i].data() + NN_INPUT_SIZE, response->states.data.data + i * NN_INPUT_SIZE);
}

// Runs on the ROS task, which waits for the episode to end
void QuteeNN::rollout_callback(const void* req, void* res){
  QuteeNN* robot = static_cast<QuteeNN*>(Qutee::instance());
  const qutee_interface__srv__Rollout_Request* weights_msg = (const qutee_interface__srv__Rollout_Request*) req;
  qutee_interface__srv__Rollout_Response* transitions = (qutee_interface__srv__Rollout_Response*) res;
  ESP_LOGI("ROS: ","Rollout requested");

  if(!robot->acquire_motors("rollout")){
    std::string reason = std::string("rollout refused: ") + (robot->motors_owner() ? robot->motors_owner() : "?")
                       + " is running; ";
    robot->set_motor_report(reason);
    // Reply with zeros: the response buffers would otherwise hold the previous rollout
    std::fill(transitions->states.data.data, transitions->states.data.data + transitions->states.data.size, 0.0f);
    std::fill(transitions->actions.data.data, transitions->actions.data.data + transitions->actions.data.size, 0.0f);
    return;
  }

  size_t nb_weights = std::min<size_t>(weights_msg->weights.data.size, robot->_policy.get_number_weights());
  if(weights_msg->weights.data.size != robot->_policy.get_number_weights())
    ESP_LOGW("ROS: ", "Received %d weights, the policy has %d", (int) weights_msg->weights.data.size,
             (int) robot->_policy.get_number_weights());
  std::copy(weights_msg->weights.data.data, weights_msg->weights.data.data + nb_weights, robot->_policy.get_weights().data());

  uint32_t begin_time = esp_log_timestamp();
  robot->run_episode();
  uint32_t duration = esp_log_timestamp() - begin_time;
  ESP_LOGI("Control Loop","Episode took %" PRIu32 " ms", duration);
  robot->release_motors();

  robot->fill_rollout_response(transitions);
}
