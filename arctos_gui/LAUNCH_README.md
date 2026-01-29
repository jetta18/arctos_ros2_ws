# Arctos GUI Launch System

Die Launch-Datei `arctos_gui.launch.py` startet das ros2_control Setup (URDF + Controller Manager + Controller) und danach die GUI.

## Verwendung

### Standard-Start (ros2_control + GUI)

```bash
ros2 launch arctos_gui arctos_gui.launch.py
```

Dies startet:
- `robot_state_publisher` (URDF/Xacro aus `arctos_description`)
- `ros2_control_node` (controller_manager + Hardware Interface)
- `joint_state_broadcaster`
- `arctos_controller`
- `arctos_gui`

## Launch-Parameter

Derzeit keine.

## Hinweise (MKS Motor Config)

- Der MKS Motor Config Tab nutzt direkte CAN-Kommunikation (`python-can` + `mks_servo_can`). Es gibt keinen separaten ROS Service Node mehr.
- Das CAN Interface (z.B. `can0`) muss ggf. vorab konfiguriert werden (500000 bitrate).

## Build & Installation

```bash
cd ~/arctos_ws
colcon build --symlink-install
source install/setup.bash
```

## Voraussetzungen

### CAN-Bus Setup (optional, nur für MKS Tab)

```bash
# CAN Interface konfigurieren
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up

# Prüfen
ip link show can0
```

### System-Dependencies

```bash
sudo apt-get install python3-pyqt5 python3-can
```

Für den MKS Tab muss außerdem das Python-Modul `mks_servo_can` (mks-servo-can Repo/Library) installiert sein.

## Troubleshooting

### GUI startet nicht

```bash
# Prüfen, ob alle Nodes laufen
ros2 node list

# Logs anschauen
ros2 launch arctos_gui arctos_gui.launch.py --screen
```

### Python-Dependency fehlt (MKS Tab)

- `ModuleNotFoundError: No module named 'can'`: `sudo apt-get install python3-can`
- `ModuleNotFoundError: No module named 'mks_servo_can'`: mks-servo-can Python Library installieren

### CAN-Bus Probleme

```bash
# CAN Interface neu starten
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up

# CAN-Nachrichten überwachen
candump can0
```

## Alternative: Nur GUI starten

Wenn ros2_control + Controller bereits laufen:

```bash
ros2 run arctos_gui arctos_gui
```
