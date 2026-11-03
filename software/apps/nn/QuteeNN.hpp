#ifndef QUTEE_NN_HPP_
#define QUTEE_NN_HPP_

// Qutee with a neural network policy (build with QUTEE_APP=nn).
// The weights come over ROS: the "rollout" service sets them, runs one episode (stand up, EPISODE_DURATION
// seconds at CONTROL_FREQUENCY Hz, back to rest) and returns the states and actions of every step.

#include "Qutee.hpp"
#include "NNQuteeController.hpp"

#include "qutee_interface/srv/rollout.h"

#define NN_INPUT_SIZE Qutee::STATE_SIZE  // 6 IMU values + 12 joint positions
#define NN_OUTPUT_SIZE Qutee::NB_MOTORS
#define NB_STEPS (CONFIG_EPISODE_DURATION * CONFIG_CONTROL_FREQUENCY)

class QuteeNN : public Qutee
{
public:
  typedef NNQuteeController<NN_INPUT_SIZE, CONFIG_NB_HIDDEN_LAYERS, CONFIG_NB_NEURONS_PER_LAYER, NN_OUTPUT_SIZE> Policy_t;
  typedef std::vector<State_t, Eigen::aligned_allocator<State_t> > List_State_t;
  typedef std::vector<Actions_t, Eigen::aligned_allocator<Actions_t> > List_Actions_t;

  QuteeNN();

  // Stand up, run the policy for NB_STEPS steps, go back to rest. The caller holds the motors.
  void run_episode();
  void control_step(State_t& state_to_fill, Actions_t& actions_to_fill);

  Policy_t& get_policy(){return _policy;}
  const List_Actions_t& get_list_actions(){return _actions;}
  const List_State_t& get_list_states(){return _states;}

protected:
  size_t ros_handles() override {return 1;}  // the rollout service
  void ros_setup(rcl_node_t* node, rclc_executor_t* executor, rclc_parameter_server_t* params) override;
  uint16_t number_weights() override {return _policy.get_number_weights();}

private:
  static void rollout_callback(const void* req, void* res);
  void init_rollout_messages();
  void fill_rollout_response(qutee_interface__srv__Rollout_Response* response);

  List_State_t _states;
  List_Actions_t _actions;
  Policy_t _policy;

  rcl_service_t _service_rollout;
  qutee_interface__srv__Rollout_Request _req_rollout;
  qutee_interface__srv__Rollout_Response _res_rollout;
};

#endif
