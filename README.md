# Qutee_v2
Repository for the base code and design of the Qutee V2 robot

See [USING_THE_ROBOT.md](USING_THE_ROBOT.md) for building, flashing and using the robot on Windows and Linux.

## Firmware applications

The firmware is one robot class, `Qutee` (in [software/main](software/main)), plus one application, chosen when building (`software/build.sh <app>`). Each application is its own binary.

| Application | What it does |
|---|---|
| [qutee](software/apps/qutee/README.md) | The base robot alone: motors, screen, menu, ROS at boot (`status`, `joint_states`). The starting point for a new application. |
| [nn](software/apps/nn/README.md) | Neural network policy; the weights come over ROS (`rollout` service). |
| [homeokinesis](software/apps/homeokinesis/README.md) | The homeokinesis controller running on the robot, with normalised joints, tunable over ROS or with its GUI. |
| [joint_reader](software/apps/joint_reader/README.md) | Torque off, publishes the joint positions, with a GUI to find each joint's range by hand. |
