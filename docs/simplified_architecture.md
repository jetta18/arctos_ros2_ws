# Vereinfachte Arctos Motor Driver & Hardware Interface Architektur

## Übersicht

Diese Dokumentation beschreibt die vereinfachte Architektur des Motor Drivers und Hardware Interface nach der Refaktorierung. Alle nicht-essentiellen Funktionen wurden entfernt, um eine saubere, CAN-Bus-freundliche Basis für die Weiterentwicklung zu schaffen.

## Hardware Interface (`arctos_hardware_interface`)

### Zweck
Das Hardware Interface ist die Schnittstelle zwischen ROS2 Control und dem Motor Driver. Es implementiert das `hardware_interface::SystemInterface` und verwaltet 6 Gelenke (X, Y, Z, A, B, C).

### Vereinfachungen
- **Entfernt**: Komplexe Logik in `read()` und `write()` Funktionen
- **Entfernt**: Polling-Thread Management
- **Entfernt**: Command change tracking und Thresholds
- **Entfernt**: Detaillierte Fehlerbehandlung und Retry-Logik

### Lifecycle States
1. **on_init()**: Validierung der Joint-Konfiguration
2. **on_configure()**: CAN-Interface Parameter laden
3. **on_activate()**: Motor Driver erstellen, Motoren konfigurieren und aktivieren
4. **on_deactivate()**: Motoren deaktivieren
5. **on_cleanup()**: Motor Driver aufräumen
6. **on_shutdown()**: Graceful shutdown
7. **on_error()**: Emergency stop

### State & Command Interfaces
- **State**: Position und Velocity für jedes Gelenk
- **Command**: Position und Velocity für jedes Gelenk
- **TODO**: `read()` und `write()` müssen neu implementiert werden

## Motor Driver Package (`arctos_motor_driver`)

Das Package wurde in **zwei separate Klassen** aufgeteilt für maximale Übersichtlichkeit:

### 1. MKSMotorDriver - Reine MKS-Protokoll-Befehle

**Datei**: `mks_motor_driver.hpp/cpp`

**Zweck**: Implementiert **NUR** die CAN-Befehle aus dem MKS Servo 42D/57D Handbuch. Keine Logik für Motor-Verwaltung, State-Management oder Joint-Mapping.

**Entfernt**:
- ❌ Motor-Konfigurationsverwaltung (addMotor, removeMotor)
- ❌ Joint-zu-Motor-Mapping
- ❌ State-Management (MotorState speichern)
- ❌ Background polling thread
- ❌ Statistik-Tracking
- ❌ Alle joint_name-basierten Wrapper-Funktionen

**Beibehalten**:
- ✅ Nur `motor_id`-basierte Funktionen (uint32_t motor_id)
- ✅ Alle MKS-Handbuch-konformen CAN-Befehle

### 2. MKSMotorManager - Motor-Verwaltung & State

**Datei**: `mks_motor_manager.hpp/cpp`

**Zweck**: Verwaltet mehrere Motoren, speichert Konfiguration und State, mapped Joint-Namen zu Motor-IDs.

**Funktionen**:
- Motor-Konfiguration (addMotor, removeMotor, getJointNames)
- Joint-zu-Motor-Mapping (findJointByMotorId, getMotorId)
- State-Management (updateEncoderData, updateSpeedData, etc.)
- Joint-State-Abfrage (getJointPosition, getJointVelocity, isJointDataValid)

### Kern-Komponenten

#### MotorConfig
Konfiguration für einen Motor:
- `motor_id`: CAN ID (1-2047)
- `hardware_type`: "MKS_42D" oder "MKS_57D"
- `gear_ratio`: Übersetzungsverhältnis
- `inverted`: Richtungsumkehr
- `working_current`: Arbeitsstrom in mA
- `holding_current`: Haltestrom in mA
- `limit_remap_enabled`: Limit-Port-Remapping
- `work_mode`: Arbeitsmodus (0-5)

#### MotorState
Aktueller Zustand eines Motors:
- `encoder`: Encoder-Daten
- `speed`: Geschwindigkeitsdaten
- `io_status`: IO-Port Status
- `motor_status`: Motor Status
- `joint_position`: Gelenkposition in Radiant
- `joint_velocity`: Gelenkgeschwindigkeit in rad/s
- `last_update`: Zeitstempel der letzten Aktualisierung
- `data_valid`: Validitäts-Flag

## MKS Command Implementation (MKSMotorDriver)

Alle Befehle verwenden **nur** `motor_id` (uint32_t) als Parameter!

### Konfiguration (Section 5.2 - Commands 0x80-0x9E)
- `calibrateMotor(motor_id)` - 0x80: Kalibrierung
- `setWorkMode(motor_id, mode)` - 0x82: Work Mode (0-5)
- `setWorkingCurrent(motor_id, current_ma)` - 0x83: Arbeitsstrom
- `setHoldingCurrentPercentage(motor_id, percentage)` - 0x9B: Haltestrom (0=10%, 8=90%)
- `setSubdivision(motor_id, subdivision)` - 0x84: Subdivision (1-256)
- `restoreDefaults(motor_id)` - 0x3F: Werkseinstellungen

### Homing (Section 5.3 - Commands 0x90-0x9E)
- `setHomeParameters(motor_id, trigger, dir, speed_rpm, enable_limit)` - 0x90
- `goHome(motor_id)` - 0x91: Homing starten
- `setZeroPosition(motor_id)` - 0x92: Aktuelle Position = 0
- `setLimitPortRemap(motor_id, enable)` - 0x9E: Limit Port Remapping

### Motor Control (Section 6.2-6.4 - Commands 0xF1-0xF7)
- `queryMotorStatus(motor_id)` - 0xF1: Status abfragen
- `enableMotor(motor_id, enable)` - 0xF3: Motor ein/aus
- `emergencyStop(motor_id)` - 0xF7: Not-Stopp
- `setVelocity(motor_id, speed_rpm, acceleration)` - 0xF6: Geschwindigkeit

### Position Control (Section 6.5-6.8 - Commands 0xFD-0xF5)
- `setRelativePositionByPulses(motor_id, pulses, speed, acc, cw)` - 0xFD
- `setAbsolutePositionByPulses(motor_id, abs_pulses, speed, acc)` - 0xFE
- `setRelativePositionByAxis(motor_id, rel_axis, speed, acc)` - 0xF4
- `setAbsolutePositionByAxis(motor_id, abs_axis, speed, acc)` - 0xF5

**Parameter-Format gemäß MKS-Handbuch**:
- `speed_rpm`: 0-3000 RPM (Mode-abhängig)
- `acceleration`: 0-255
- `pulses`: 0-0xFFFFFF (24-bit unsigned)
- `axis`: -8388607 bis +8388607 (24-bit signed)

### State Reading (Section 5.1 - Commands 0x30-0x3E)
- `requestEncoderCarry(motor_id)` - 0x30: Encoder mit Carry
- `requestEncoderReading(motor_id)` - 0x31: Encoder Addition
- `requestSpeedReading(motor_id)` - 0x32: Geschwindigkeit
- `requestPulseCount(motor_id)` - 0x33: Pulszahl
- `requestIOStatus(motor_id)` - 0x34: IO-Port Status
- `requestAngleError(motor_id)` - 0x39: Winkelfehler
- `requestEnableStatus(motor_id)` - 0x3A: Enable-Pin Status

### CAN Message Processing
- `processCANMessage(msg)`: Weiterleitung an MKSMotorManager

## Motor Manager Functions (MKSMotorManager)

### Konfigurationsverwaltung
- `addMotor(joint_name, config)`: Motor hinzufügen
- `removeMotor(joint_name)`: Motor entfernen
- `getJointNames()`: Liste aller Joints
- `getMotorConfig(joint_name)`: Konfiguration abrufen
- `getMotorId(joint_name)`: Motor-ID für Joint
- `findJointByMotorId(motor_id)`: Joint-Name für Motor-ID

### State-Management
- `updateEncoderData(motor_id, encoder)`: Encoder-Daten aktualisieren
- `updateSpeedData(motor_id, speed)`: Geschwindigkeitsdaten
- `updateIOStatus(motor_id, io_status)`: IO-Status
- `updateMotorStatus(motor_id, motor_status)`: Motor-Status

### State-Abfrage
- `getJointPosition(joint_name)`: Position in Radiant
- `getJointVelocity(joint_name)`: Geschwindigkeit in rad/s
- `isJointDataValid(joint_name, max_age)`: Validitätsprüfung
- `getMotorState(joint_name)`: Kompletter State

## CAN-Bus Konfiguration

### Hardware
- **CAN Adapter**: MKS Canable 2.0 Pro
- **Bitrate**: 500 kbit/s (0x8A: 0x02)
- **Interface**: can0
- **Terminierung**: 120Ω an beiden Enden (bei Multi-Slave)

### CAN Frame Format
- **Standard Frame** (11-bit ID)
- **CAN ID**: 1-2047 (0 = Broadcast)
- **DLC**: 2-8 Bytes
- **Checksum**: 8-bit (alle Bytes XOR)

## TODO: Neue Architektur

### Nächste Schritte
1. **Hardware Interface `read()`**: 
   - Implementierung einer effizienten State-Lesestrategie
   - Synchrone oder asynchrone Encoder-Abfrage?

2. **Hardware Interface `write()`**:
   - Welcher Position-Mode ist optimal? (0xFD, 0xFE, 0xF4, oder 0xF5)
   - Geschwindigkeits-Feedforward implementieren?
   - Trajectory-Interpolation?

3. **Control Loop Design**:
   - Welche Regelfrequenz? (100Hz, 200Hz, 500Hz, 1000Hz?)
   - Position-Only oder Position+Velocity Control?
   - Wie werden Encoder-Updates abgerufen?

4. **CAN-Bus Optimierung**:
   - Batch-Requests für mehrere Motoren?
   - Inter-Frame-Delay notwendig?
   - Response-Handling synchron oder asynchron?

## Offene Fragen

1. **Position Control Mode**: Welcher der 4 Modi (0xFD-0xF5) ist am besten für Trajectory Following?
2. **Polling Strategy**: Aktives Polling oder Event-driven?
3. **Error Handling**: Wie mit fehlenden Encoder-Updates umgehen?
4. **Synchronization**: Wie werden alle 6 Motoren synchronisiert?

## Referenzen
- MKS Servo Manual: `/home/michael/arctos_ros2_ws/src/docs/mks_servo_reference.md`
- CAN Bitrate: 500 kbit/s
- Work Mode: SR_vFOC (Mode 5) für serielle Steuerung mit Encoder
