# MKS Motor Driver Configuration System

Dieses System ermöglicht die Konfiguration und Datenabfrage der MKS Servo 42D/57D Motortreiber über CAN-Bus mit einer modernen GUI-Integration.

## Architektur

```
┌─────────────────┐         ┌──────────────────┐         ┌─────────────┐
│   Arctos GUI    │ ◄─────► │ MKS Config       │ ◄─────► │  CAN Bus    │
│   (PyQt5)       │  ROS2   │ Service Node     │  Socket │  (can0)     │
│                 │ Service │  (C++)           │   CAN   │             │
└─────────────────┘         └──────────────────┘         └─────────────┘
                                                                 │
                                                                 ▼
                                                          ┌─────────────┐
                                                          │ MKS Servo   │
                                                          │ 42D/57D     │
                                                          └─────────────┘
```

## Komponenten

### 1. ROS2 Service Definitionen

**`srv/MKSMotorConfig.srv`** - Konfigurationsservice
- Kalibrierung
- Arbeitsmodus (Work Mode)
- Stromeinstellungen (Working/Holding Current)
- Subdivision (Microsteps)
- Homing-Parameter
- Motor Enable/Disable

**`srv/MKSMotorRead.srv`** - Datenabfrage-Service
- Encoder-Position
- Geschwindigkeit
- IO-Status
- Motor-Status

### 2. MKS Config Service Node (`mks_config_service`)

C++ Node, der die ROS2 Services bereitstellt und mit dem CAN-Bus kommuniziert.

**Parameter:**
- `can_interface` (default: "can0") - CAN Interface Name

**Services:**
- `/mks_motor_config` - Konfigurationsservice
- `/mks_motor_read` - Datenabfrage-Service

### 3. GUI Komponenten

**`MKSConfigClient`** - ROS2 Client-Node (Python)
- Verbindet sich mit den MKS Config Services
- Stellt Python-API für GUI bereit

**`MKSConfigWidget`** - PyQt5 Widget
- Moderne, professionelle Benutzeroberfläche
- Tabs für verschiedene Konfigurationsbereiche:
  - **Basic Configuration**: Work Mode, Kalibrierung, Stromeinstellungen
  - **Advanced Settings**: Subdivision, Limit Switches, Factory Reset
  - **Homing**: Homing-Parameter und -Aktionen
  - **Motor Control**: Enable/Disable, Status-Abfrage
  - **Read Data**: Encoder, Speed, IO Status

## Installation & Build

### 1. Dependencies installieren

```bash
# System-Dependencies
sudo apt-get install python3-pyqt5

# CAN-Bus Setup (falls noch nicht konfiguriert)
sudo ip link set can0 type can bitrate 500000
sudo ip link set up can0
```

### 2. Packages bauen

```bash
cd ~/arctos_ros2_ws
colcon build --packages-select arctos_motor_driver arctos_gui
source install/setup.bash
```

## Verwendung

### 1. MKS Config Service starten

```bash
# Mit Standard CAN Interface (can0)
ros2 launch arctos_motor_driver mks_config_service.launch.py

# Mit anderem CAN Interface
ros2 launch arctos_motor_driver mks_config_service.launch.py can_interface:=can1
```

### 2. GUI starten

```bash
ros2 run arctos_gui arctos_gui
```

Die GUI öffnet sich mit einem neuen Tab "MKS Motor Config", der alle Konfigurationsmöglichkeiten bietet.

### 3. Direkte Service-Aufrufe (optional)

```bash
# Motor kalibrieren
ros2 service call /mks_motor_config arctos_motor_driver/srv/MKSMotorConfig \
  "{motor_id: 1, command_type: 'calibrate'}"

# Work Mode setzen (SR_vFOC = Mode 5)
ros2 service call /mks_motor_config arctos_motor_driver/srv/MKSMotorConfig \
  "{motor_id: 1, command_type: 'set_work_mode', work_mode: 5}"

# Working Current setzen (1600 mA)
ros2 service call /mks_motor_config arctos_motor_driver/srv/MKSMotorConfig \
  "{motor_id: 1, command_type: 'set_current', working_current_ma: 1600}"

# Encoder auslesen
ros2 service call /mks_motor_read arctos_motor_driver/srv/MKSMotorRead \
  "{motor_id: 1, read_type: 'encoder'}"
```

## GUI Features

### Basic Configuration Tab
- **Work Mode Auswahl**: 6 verschiedene Modi (CR, CR_vFOC, SR_OPEN, SR_CLOSE, SR_FOC, SR_vFOC)
- **Working Current**: 100-3000 mA einstellbar
- **Holding Current**: 0-100% einstellbar
- **Motor Calibration**: Ein-Klick-Kalibrierung

### Advanced Settings Tab
- **Microstep Subdivision**: 0-8 (Full Step bis 256 Microsteps)
- **Limit Port Remapping**: Enable/Disable
- **Factory Reset**: Alle Parameter auf Werkseinstellungen zurücksetzen

### Homing Tab
- **Trigger Level**: Low/High Level
- **Direction**: Clockwise/Counter-Clockwise
- **Speed**: 1-3000 RPM
- **Enable Limit Switch**: Ja/Nein
- **Homing Sequence starten**
- **Zero Position setzen**

### Motor Control Tab
- **Enable/Disable Motor**: Große, gut sichtbare Buttons
- **Query Motor Status**: Aktuellen Status abfragen

### Read Data Tab
- **Read Encoder**: Aktuelle Position auslesen
- **Read Speed**: Aktuelle Geschwindigkeit auslesen
- **Read IO Status**: Limit Switches und Stall Detection
- **Read Motor Status**: Vollständiger Motor-Status

### Status Log
- Alle Aktionen werden im Status-Log protokolliert
- Farbcodierung: Grün = Erfolg, Rot = Fehler
- Zeitstempel für jede Aktion

## Empfohlene Konfiguration für Arctos Robot Arm

```bash
# 1. Motor kalibrieren
Motor ID: 1
Command: Calibrate Motor

# 2. Work Mode setzen
Work Mode: 5 - SR_vFOC (Closed Loop, Speed Control, FOC + Encoder)

# 3. Stromeinstellungen
Working Current: 1600 mA
Holding Current: 70%

# 4. Subdivision
Subdivision: 7 - 128 Microsteps

# 5. Homing-Parameter (optional)
Trigger Level: 0 - Low Level
Direction: 1 - Counter-Clockwise
Speed: 300 RPM
Enable Limit: Yes
```

## Troubleshooting

### Service nicht verfügbar
```bash
# Prüfen, ob Service läuft
ros2 service list | grep mks

# Service-Node neu starten
ros2 launch arctos_motor_driver mks_config_service.launch.py
```

### CAN-Bus Fehler
```bash
# CAN Interface prüfen
ip link show can0

# CAN Interface neu konfigurieren
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up

# CAN-Nachrichten überwachen
candump can0
```

### Motor antwortet nicht
1. Motor-ID prüfen (Standard: 1)
2. CAN-Bus Verkabelung prüfen
3. Motor-Stromversorgung prüfen
4. CAN-Bus Terminierung prüfen (120Ω)

## Technische Details

### Work Modes
- **0 - CR**: Closed Loop, Current Control
- **1 - CR_vFOC**: Closed Loop, Current Control, FOC
- **2 - SR_OPEN**: Open Loop, Speed Control
- **3 - SR_CLOSE**: Closed Loop, Speed Control
- **4 - SR_FOC**: Closed Loop, Speed Control, FOC
- **5 - SR_vFOC**: Closed Loop, Speed Control, FOC + Encoder (empfohlen)

### Subdivision (Microsteps)
- **0**: Full Step (200 steps/rev)
- **1**: 2 Microsteps (400 steps/rev)
- **2**: 4 Microsteps (800 steps/rev)
- **3**: 8 Microsteps (1600 steps/rev)
- **4**: 16 Microsteps (3200 steps/rev)
- **5**: 32 Microsteps (6400 steps/rev)
- **6**: 64 Microsteps (12800 steps/rev)
- **7**: 128 Microsteps (25600 steps/rev) - empfohlen
- **8**: 256 Microsteps (51200 steps/rev)

### CAN-Bus Spezifikationen
- **Bitrate**: 500 kbit/s
- **Protokoll**: MKS Servo 42D/57D Protokoll v1.0.4
- **Terminierung**: 120Ω an beiden Enden
- **Max. Kabellänge**: 40m @ 500 kbit/s

## Weiterführende Dokumentation

- [MKS Servo Manual](../docs/mks_servo_reference.md)
- [Arctos Motor Driver README](README.md)
- [CAN-Bus Setup Guide](../docs/can_bus_setup.md)
