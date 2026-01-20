# Arctos Motor Driver Package

Dieses Package enthält die CAN-Bus Kommunikation für MKS Servo 42D/57D Motoren.

## Architektur

Das Package ist in **zwei separate Komponenten** aufgeteilt:

### 1. MKSMotorDriver (mks_motor_driver.hpp/cpp)

**Zweck**: Reine MKS-Protokoll-Implementierung

Implementiert **NUR** die CAN-Befehle aus dem offiziellen MKS Servo 42D/57D Handbuch (Version 1.0.4).

- ✅ Alle Funktionen verwenden `motor_id` (uint32_t) als Parameter
- ✅ Keine Abstraktion, kein State-Management
- ✅ Direkte 1:1 Mapping zu MKS CAN-Befehlen

**Verwendung**:
```cpp
auto driver = std::make_shared<MKSMotorDriver>(node, "can0");

// Konfiguration (motor_id basiert)
driver->calibrateMotor(1);                    // Motor ID 1
driver->setWorkMode(1, 5);                    // SR_vFOC Mode
driver->setWorkingCurrent(1, 1600);           // 1600 mA
driver->enableMotor(1, true);                 // Enable

// Position Control
driver->setAbsolutePositionByAxis(1, 16384, 600, 10);  // 90°, 600 RPM, acc=10

// State Reading
driver->requestEncoderReading(1);             // Request encoder
driver->requestSpeedReading(1);               // Request speed
```

### 2. MKSMotorManager (mks_motor_manager.hpp/cpp)

**Zweck**: Motor-Verwaltung und State-Management

Verwaltet mehrere Motoren, Joint-Namen, Konfigurationen und States.

- Motor-Konfiguration und Joint-Mapping
- CAN Response Processing
- State Caching und Conversion (Encoder → Radiant)
- Gear Ratio und Inversion Handling

**Verwendung**:
```cpp
auto driver = std::make_shared<MKSMotorDriver>(node, "can0");
auto manager = std::make_unique<MKSMotorManager>(driver);

// Motor hinzufügen mit Joint-Namen
MotorConfig config;
config.motor_id = 1;
config.gear_ratio = 5.0;
config.inverted = false;
config.working_current = 1600;
config.work_mode = 5;  // SR_vFOC

manager->addMotor("X_joint", config);

// Joint-basierte Abfragen
uint32_t motor_id = manager->getMotorId("X_joint");
double position = manager->getJointPosition("X_joint");    // in Radiant
double velocity = manager->getJointVelocity("X_joint");    // in rad/s
bool valid = manager->isJointDataValid("X_joint", 1.0);

// CAN Response Processing
manager->updateEncoderData(motor_id, encoder_data);
manager->updateSpeedData(motor_id, speed_data);
```

## MKS Command Reference

Alle Commands sind nach dem MKS Handbuch strukturiert:

### Configuration (Section 5.2)
- `0x80` - Calibrate Motor
- `0x82` - Set Work Mode (0-5)
- `0x83` - Set Working Current
- `0x84` - Set Subdivision
- `0x9B` - Set Holding Current %
- `0x3F` - Restore Defaults

### Homing (Section 5.3)
- `0x90` - Set Home Parameters
- `0x91` - Go Home
- `0x92` - Set Zero Position
- `0x9E` - Set Limit Port Remap

### Motor Control (Section 6.2-6.4)
- `0xF1` - Query Motor Status
- `0xF3` - Enable/Disable Motor
- `0xF6` - Speed Mode
- `0xF7` - Emergency Stop

### Position Control (Section 6.5-6.8)
- `0xFD` - Relative Position by Pulses
- `0xFE` - Absolute Position by Pulses
- `0xF4` - Relative Position by Axis
- `0xF5` - Absolute Position by Axis

### Read Parameters (Section 5.1)
- `0x30` - Read Encoder (Carry)
- `0x31` - Read Encoder (Addition)
- `0x32` - Read Speed
- `0x33` - Read Pulse Count
- `0x34` - Read IO Status
- `0x39` - Read Angle Error
- `0x3A` - Read Enable Status

## Hardware Setup

- **Motors**: MKS Servo 42D / 57D
- **CAN Adapter**: MKS Canable 2.0 Pro
- **Bitrate**: 500 kbit/s
- **Interface**: can0
- **Work Mode**: SR_vFOC (Mode 5) für serielle Steuerung mit Encoder

## Dependencies

- ROS2 Humble
- rclcpp
- can_msgs
- sensor_msgs
- Linux SocketCAN

## Build

```bash
cd ~/arctos_ros2_ws
colcon build --packages-select arctos_motor_driver
source install/setup.bash
```

## Nächste Schritte

1. **Hardware Interface Integration**: Verwende MKSMotorManager im Hardware Interface
2. **Control Strategy**: Implementiere read() und write() Logik
3. **Testing**: Teste alle Position Control Modes mit echten Motoren

## Dokumentation

- [MKS Servo Manual](../docs/mks_servo_reference.md)
- [Simplified Architecture](../docs/simplified_architecture.md)
