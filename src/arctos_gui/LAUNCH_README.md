# Arctos GUI Launch System

Die Arctos GUI kann jetzt über eine Launch-Datei gestartet werden, die automatisch alle benötigten Services mit startet.

## Verwendung

### Standard-Start (mit MKS Config Service)

```bash
ros2 launch arctos_gui arctos_gui.launch.py
```

Dies startet:
- **MKS Config Service** auf `can0`
- **Arctos GUI** mit allen Komponenten

### Mit anderem CAN Interface

```bash
ros2 launch arctos_gui arctos_gui.launch.py can_interface:=can1
```

### Ohne MKS Config Service

```bash
ros2 launch arctos_gui arctos_gui.launch.py enable_mks_config:=false
```

## Launch-Parameter

| Parameter | Default | Beschreibung |
|-----------|---------|--------------|
| `can_interface` | `can0` | CAN Interface für MKS Motor Driver |
| `enable_mks_config` | `true` | MKS Config Service aktivieren/deaktivieren |

## Komponenten

Die Launch-Datei startet folgende Nodes:

1. **mks_config_service** (optional)
   - Package: `arctos_motor_driver`
   - Stellt Services für MKS Motor Konfiguration bereit
   - Verbindet sich mit CAN-Bus

2. **arctos_gui**
   - Package: `arctos_gui`
   - PyQt5 GUI mit folgenden Tabs:
     - Jog Control
     - STM32 Debug
     - MKS Motor Config

## Build & Installation

```bash
cd ~/arctos_ros2_ws
colcon build --packages-select arctos_gui arctos_motor_driver
source install/setup.bash
```

## Voraussetzungen

### CAN-Bus Setup

```bash
# CAN Interface konfigurieren
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up

# Prüfen
ip link show can0
```

### System-Dependencies

```bash
sudo apt-get install python3-pyqt5
```

## Troubleshooting

### GUI startet nicht

```bash
# Prüfen, ob alle Nodes laufen
ros2 node list

# Logs anschauen
ros2 launch arctos_gui arctos_gui.launch.py --screen
```

### MKS Config Service Fehler

```bash
# Nur GUI starten (ohne MKS Service)
ros2 launch arctos_gui arctos_gui.launch.py enable_mks_config:=false

# Service separat starten
ros2 launch arctos_motor_driver mks_config_service.launch.py
```

### CAN-Bus Probleme

```bash
# CAN Interface neu starten
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up

# CAN-Nachrichten überwachen
candump can0
```

## Vorteile der Launch-Datei

✅ **Ein Befehl** - Startet alle benötigten Komponenten  
✅ **Konfigurierbar** - Parameter für verschiedene Setups  
✅ **Konsistent** - Immer die gleiche Startreihenfolge  
✅ **Wartbar** - Zentrale Konfiguration  
✅ **Flexibel** - Services können optional aktiviert werden  

## Alternative: Einzelne Komponenten starten

Falls gewünscht, können die Komponenten auch einzeln gestartet werden:

```bash
# Terminal 1: MKS Config Service
ros2 run arctos_motor_driver mks_config_service --ros-args -p can_interface:=can0

# Terminal 2: GUI
ros2 run arctos_gui arctos_gui
```
