# Arctos Hardware Interface - STM32 Stepper Control

Clean and efficient ROS2 hardware interface for STM32-based 6-axis stepper motor control.

## Features

- **Direct TCP/IP Communication**: Binary protocol for low-latency control
- **JTC Streaming**: Joint Trajectory Controller compatible at 100-250Hz
- **6-Axis Control**: Position and velocity command/state interfaces
- **Minimal Dependencies**: Only core ROS2 Control dependencies
- **Clean Architecture**: Single responsibility, modular design

## Architecture

```
ROS2 Control Stack
       ↓
STM32StepperInterface (Hardware Interface)
       ↓
TCP/IP Binary Protocol
       ↓
STM32H755 @ 480MHz
       ↓
6x Stepper Motors (MKS Servo 42/57D)
```

## Protocol

### Commands
- `CMD_JTC_STREAM (0x01)`: ROS2 JTC streaming (position, velocity)
- `CMD_SET_DIRECT (0x02)`: Direct 6-axis control
- `CMD_MOVE_SINGLE (0x03)`: Single axis control (debugging)
- `CMD_GET_STATE (0x10)`: Query current state

### Responses
- `RESP_OK (0x00)`: Success
- `RESP_ERROR (0x01)`: Error
- `RESP_STATE (0x02)`: State data

## Configuration

### URDF/XACRO Example

```xml
<ros2_control name="arctos_system" type="system">
  <hardware>
    <plugin>arctos_hardware_interface/STM32StepperInterface</plugin>
    <param name="stm32_host">192.168.178.159</param>
    <param name="stm32_port">8888</param>
  </hardware>
  
  <joint name="joint1">
    <command_interface name="position"/>
    <command_interface name="velocity"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
  
  <!-- Repeat for joint2-joint6 -->
</ros2_control>
```

## Building

```bash
cd ~/arctos_ros2_ws
colcon build --packages-select arctos_hardware_interface
source install/setup.bash
```

## Usage

### With ROS2 Control

```bash
ros2 launch arctos_bringup arctos_control.launch.py
```

### Testing with GUI

```bash
# On STM32 side - ensure firmware is running

# On PC side - test single axis control
python3 gui_axis_control.py
```

## Implementation Details

### Real-time Loop

1. **Read Phase**: Query STM32 for current position/velocity
2. **Write Phase**: Send JTC command with target position/velocity
3. **Frequency**: Typically 100-250Hz (configurable via controller)

### Thread Safety

- Socket operations are blocking with 2s timeout
- No internal threading - relies on ROS2 Control's real-time loop
- State/command vectors are accessed only in read/write methods

### Error Handling

- Connection failures logged and return ERROR
- Timeout on socket operations (2s)
- Automatic reconnection on deactivate/activate cycle

## Performance

- **Latency**: ~1-2ms per command (TCP/IP + processing)
- **Throughput**: Tested at 250Hz streaming
- **CPU Load**: <5% on modern x86_64 systems

## Dependencies

- `rclcpp`: ROS2 C++ client library
- `hardware_interface`: ROS2 Control hardware interface
- `pluginlib`: Plugin management
- `rclcpp_lifecycle`: Lifecycle node support

## Future Extensions

- Endstop handling
- Homing sequences
- Motor status monitoring
- Error recovery strategies
- Configuration via parameters

## License

TODO: Add license

## Maintainer

Michael (michael@example.com)
