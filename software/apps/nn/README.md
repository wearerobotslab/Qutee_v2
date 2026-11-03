# Neural network controller (`nn`)

A neural network policy whose weights come over ROS: each `rollout` call sets the weights, runs one episode and returns what happened. It's meant for learning from a PC, for example with an evolutionary algorithm.

| File | What it is |
|---|---|
| `QuteeNN.hpp`, `QuteeNN.cpp` | `QuteeNN`, derived from `Qutee`: the policy, the episode and the `rollout` service |
| `NNQuteeController.hpp` | The neural network |
| `main.cpp` | Creates a `QuteeNN` and runs it |

## The policy

18 inputs, `NB_HIDDEN_LAYERS` hidden layers of `NB_NEURONS_PER_LAYER` neurons (1 x 10 by default) and 12 outputs, all with tanh. Inputs: orientation x, y, z (rad) and linear acceleration x, y, z (m/s²) from the IMU (zeros without one), then the 12 joint positions. Outputs: the 12 joint targets. Joints and targets use the firmware's scale: 0 = standing pose, 1 = 45°. With the default sizes there are 322 weights; `status` reports the number. The weights' layout is in [USING_THE_ROBOT.md](../../../USING_THE_ROBOT.md#rollout-data).

Settings in menuconfig, **QUTEE settings → Neural network controller**: hidden layers, neurons per layer, episode duration (5 s by default). The control frequency (50 Hz) is in **QUTEE settings**.

## ROS

Everything from the [base robot](../qutee/README.md) (`status`, `joint_states`, `torque_off_at_rest`), plus:

| | |
|---|---|
| `/<name>/rollout` service | `qutee_interface/srv/Rollout`: send the weights; the robot stands up, runs the episode, goes back to rest, and replies with the states (steps x 18) and actions (steps x 12). About 10 s. |

A rollout is refused, with zeros and a message in `status`, while the checkup runs. Check `status`'s `error_message` after each rollout: it reports motor faults and incomplete position reads.

There are no menu entries beyond the base robot's.

## Build, flash and try

From `software/`, in WSL or Linux:

```bash
./build.sh nn          # -> build/nn/qutee_nn.bin
```

Flash from `build/nn` (see [USING_THE_ROBOT.md](../../../USING_THE_ROBOT.md#flash)). Calling the services needs the `qutee_interface` package; [Calling the services](../../../USING_THE_ROBOT.md#calling-the-services) shows how, with a rollout example in Python. Large weights (±1) make the joints swing at full speed and can overload a motor.
