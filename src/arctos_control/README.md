# arctos_control

Tools and launch files to operate Arctos with topic_based_ros2_control and micro-ROS (ESP32).

## Overview
- Provides a launch to run ros2_control with `joint_state_topic_hardware_interface/JointStateTopicSystem` via Xacro args.
- Simulation node `sim_esp32` publishes `sensor_msgs/JointState` on a configurable `joint_states_topic` and listens for commands on `joint_commands_topic`.
- `watchdog` monitors command stream and publishes an E-STOP boolean when command timeout is exceeded.

## Topics
- joint_commands_topic (default `/topic_based_joint_commands`): sensor_msgs/JointState
- joint_states_topic (default `/topic_based_joint_states`): sensor_msgs/JointState

## Launch
```
ros2 launch arctos_control topic_based_bringup.launch.py use_topic_based:=true \
  joint_commands_topic:=/topic_based_joint_commands joint_states_topic:=/topic_based_joint_states
```

## ESP32 (micro-ROS) sketch outline
- Subscribe to `joint_commands_topic` (sensor_msgs/JointState)
- Publish `joint_states_topic` (sensor_msgs/JointState)
- Ensure QoS RELIABLE, depth >= 10
- Add heartbeat and command timeout handling (optional: obey `/arctos/estop` Bool)
