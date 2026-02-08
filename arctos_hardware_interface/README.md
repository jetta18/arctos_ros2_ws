# Arctos Hardware Interface

ROS 2 Control hardware interface for the Arctos 6-axis robot arm, communicating with an STM32H755 controller over UDP.

## Architecture

```
MoveIt / RViz
       ↓
ArctosSegmentController (FollowJointTrajectory action)
       ↓
STM32HardwareInterface (ros2_control SystemInterface)
       ↓  UDP binary protocol
STM32H755 @ 480 MHz
       ↓
6× Stepper Motors (MKS Servo 42D/57D)
```

The hardware interface owns the UDP socket. Trajectories are uploaded out-of-band by the controller via a shared protocol pointer. `read()` polls `GET_STATE` at the controller manager rate (100 Hz). `write()` is a no-op.

## Plugin

- **Class**: `arctos_hardware_interface/STM32HardwareInterface`
- **Type**: `hardware_interface::SystemInterface`

## Protocol Commands

- `CMD_PING (0x20)` — connectivity check
- `CMD_GET_STATE (0x10)` — positions, velocities, system state
- `CMD_STOP (0x03)` — emergency stop
- `CMD_TRAJ_BEGIN (0x40)` — start trajectory upload
- `CMD_TRAJ_POINT (0x41)` — send trajectory point (fire-and-forget)
- `CMD_TRAJ_EXECUTE (0x42)` — execute uploaded trajectory
- `CMD_TRAJ_CANCEL (0x43)` — cancel running trajectory

## Building

```bash
cd ~/arctos_ws
colcon build --packages-select arctos_hardware_interface --symlink-install
```

## Dependencies

- `rclcpp`, `hardware_interface`, `pluginlib`, `rclcpp_lifecycle`
