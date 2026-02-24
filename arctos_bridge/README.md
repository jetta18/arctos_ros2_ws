# arctos_bridge

Communication bridge between the Arctos STM32 firmware and ROS2.

Receives state broadcasts from the STM32 over UDP and publishes them as ROS2 topics. Provides ROS2 services for direct STM32 commands.

## Architecture

```
STM32 (UDP 8889) ──broadcast──> arctos_bridge_node ──> /arctos/state
                                                   ──> /arctos/endstops
                                                   ──> /joint_states

ROS2 service call ──> arctos_bridge_node ──cmd──> STM32 (UDP 8888)
```

The node runs a background thread that listens for UDP broadcast packets from the STM32. Each packet is parsed, converted from steps to radians, and published on ROS2 topics. Services forward commands to the STM32 command port.

## Usage

```bash
ros2 launch arctos_bridge arctos_bridge.launch.xml
```

Or with a custom config:

```bash
ros2 launch arctos_bridge arctos_bridge.launch.xml config_file:=/path/to/custom.yaml
```

## Topics (Published)

| Topic | Type | Rate | Description |
|---|---|---|---|
| `/arctos/state` | `arctos_msgs/ArctosState` | Configurable (default 100 Hz) | Full robot state |
| `/arctos/endstops` | `arctos_msgs/ArctosEndstops` | Same as state | Endstop details |
| `/joint_states` | `sensor_msgs/JointState` | Same as state | Standard joint state for TF/RViz |

## Services

| Service | Type | Description |
|---|---|---|
| `/arctos/ping` | `arctos_msgs/Ping` | Check STM32 connectivity |
| `/arctos/stop` | `arctos_msgs/Stop` | Emergency stop |
| `/arctos/set_servo` | `arctos_msgs/SetServo` | Set gripper servo |
| `/arctos/home_axis` | `arctos_msgs/HomeAxis` | Start/stop axis homing |

## Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `stm32_host` | string | `192.168.178.159` | STM32 IP address |
| `stm32_command_port` | int | `8888` | Command UDP port |
| `stm32_broadcast_port` | int | `8889` | Broadcast UDP port |
| `broadcast_listen_port` | int | `9000` | Local port for receiving broadcasts |
| `broadcast_rate_hz` | int | `100` | Requested broadcast rate (1-200 Hz) |
| `socket_timeout_s` | float | `2.0` | UDP socket timeout |
| `joint_names` | string[] | `[X_joint, ..., C_joint]` | Joint names matching URDF |
| `steps_per_rev` | int | `200` | Motor steps per revolution |
| `microsteps` | int | `16` | Microstepping setting |
| `gear_ratios` | float[] | `[13.5, 150, 150, 48, 27.3375, 10]` | Per-joint gear ratios |
| `joint_inversions` | bool[] | `[F, F, F, T, F, T]` | Per-joint direction inversion |

## Actions

| Action | Type | Description |
|---|---|---|
| `/arctos_controller/follow_joint_trajectory` | `control_msgs/FollowJointTrajectory` | Upload and execute trajectory on STM32 |

The action server handles the full trajectory lifecycle: joint name mapping, unit conversion (rad→steps), point upload, execution, and completion monitoring via the state broadcast stream.

## Dependencies

- `arctos_msgs`
- `sensor_msgs`
- `control_msgs`
- `trajectory_msgs`
- `rclpy`
