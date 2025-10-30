# STM32 Joint Commands Subscriber Implementation

## Übersicht

Diese Implementierung ermöglicht es dem STM32 H755ZiQ, Gelenkwinkel vom ROS2-Topic `/topic_based_joint_commands` zu empfangen und zu verarbeiten.

## Implementierte Funktionen

### 1. Datenstruktur (`JointData_t`)
```c
typedef struct {
  double position[6];      // Gelenkpositionen in Radiant (X, Y, Z, A, B, C)
  double velocity[6];      // Gelenkgeschwindigkeiten in rad/s
  uint32_t last_update_time; // Zeitstempel der letzten Aktualisierung
  bool data_received;      // Flag, ob Daten empfangen wurden
} JointData_t;
```

### 2. Callback-Funktion (`joint_commands_callback`)
Diese Funktion wird automatisch aufgerufen, wenn neue Daten auf dem Topic `/topic_based_joint_commands` ankommen:
- Extrahiert Gelenkpositionen und -geschwindigkeiten aus der Nachricht
- Aktualisiert den Zeitstempel
- Setzt das `data_received` Flag

### 3. Verarbeitungsfunktion (`process_joint_commands`)
Diese Funktion verarbeitet die empfangenen Gelenkwinkel:
- Prüft auf Timeout (1 Sekunde)
- Iteriert über alle 6 Gelenke
- Stellt Position und Geschwindigkeit für jedes Gelenk bereit

## Gelenk-Mapping

Die Gelenkwinkel werden in folgender Reihenfolge empfangen:
- `joint_data.position[0]` = X_joint (Basis-Rotation)
- `joint_data.position[1]` = Y_joint
- `joint_data.position[2]` = Z_joint
- `joint_data.position[3]` = A_joint
- `joint_data.position[4]` = B_joint
- `joint_data.position[5]` = C_joint (End-Effektor)

## Integration in Ihre Motorsteuerung

Um die empfangenen Gelenkwinkel zu nutzen, müssen Sie die `process_joint_commands()` Funktion erweitern:

### Beispiel 1: Umrechnung in Encoder-Counts
```c
#define ENCODER_COUNTS_PER_RAD 1000.0  // Anpassen für Ihre Motoren

void process_joint_commands(void)
{
  if (!joint_data.data_received) return;
  if ((HAL_GetTick() - joint_data.last_update_time) > 1000) return;
  
  for (int i = 0; i < 6; i++) {
    double position_rad = joint_data.position[i];
    int32_t encoder_counts = (int32_t)(position_rad * ENCODER_COUNTS_PER_RAD);
    
    // Senden Sie den Befehl an Ihren Motor-Controller
    send_motor_command(i, encoder_counts);
  }
}
```

### Beispiel 2: CAN-Bus Kommunikation mit MKS Servos
```c
void process_joint_commands(void)
{
  if (!joint_data.data_received) return;
  if ((HAL_GetTick() - joint_data.last_update_time) > 1000) return;
  
  // Gear Ratios aus ros2_control.xacro
  const double gear_ratios[6] = {13.5, 150.0, 150.0, 48.0, 27.3375, 10.0};
  
  for (int i = 0; i < 6; i++) {
    double position_rad = joint_data.position[i];
    
    // Umrechnung in Motor-Position (mit Gear Ratio)
    double motor_position = position_rad * gear_ratios[i];
    
    // Umrechnung in Encoder-Counts (MKS Servos: 16384 counts/rev)
    int32_t encoder_counts = (int32_t)((motor_position / (2.0 * M_PI)) * 16384.0);
    
    // CAN-Nachricht senden
    send_mks_position_command(i + 1, encoder_counts);  // Motor IDs: 1-6
  }
}
```

### Beispiel 3: PWM-Steuerung
```c
void process_joint_commands(void)
{
  if (!joint_data.data_received) return;
  if ((HAL_GetTick() - joint_data.last_update_time) > 1000) return;
  
  for (int i = 0; i < 6; i++) {
    double position_rad = joint_data.position[i];
    
    // Umrechnung in PWM Duty Cycle (0-100%)
    // Annahme: -π bis +π entspricht 0-100% Duty Cycle
    double duty_cycle = ((position_rad + M_PI) / (2.0 * M_PI)) * 100.0;
    duty_cycle = fmax(0.0, fmin(100.0, duty_cycle));  // Clamp auf 0-100%
    
    // PWM setzen
    set_pwm_duty_cycle(i, duty_cycle);
  }
}
```

## Hauptschleife

Die Hauptschleife in `StartDefaultTask` führt folgende Schritte aus:
1. **Executor Spin** (100ms): Verarbeitet eingehende ROS2-Nachrichten
2. **Status-Publish**: Sendet einen Heartbeat auf `/stm32_status`
3. **Joint Processing**: Ruft `process_joint_commands()` auf
4. **Delay**: 10ms Pause

## Konfiguration

### Topic-Name
Der Subscriber ist auf das Topic `/topic_based_joint_commands` konfiguriert, wie in `ros2_control.xacro` definiert.

### Update-Rate
- ROS2 Control Update Rate: 100 Hz (aus `ros2_controllers.yaml`)
- STM32 Loop Rate: ~100 Hz (10ms Delay)
- Executor Timeout: 100ms

### Timeout-Handling
Die Implementierung prüft auf veraltete Daten (Timeout: 1 Sekunde). Bei einem Timeout werden keine Motorbefehle gesendet.

## Debugging

### Status-Topic
Der STM32 publiziert einen Zähler auf `/stm32_status`, um die Verbindung zu überwachen:
```bash
ros2 topic echo /stm32_status
```

### Joint-Daten prüfen
Sie können die empfangenen Daten im Code ausgeben:
```c
if (joint_data.data_received) {
  printf("X: %.4f, Y: %.4f, Z: %.4f, A: %.4f, B: %.4f, C: %.4f\n",
         joint_data.position[0], joint_data.position[1], joint_data.position[2],
         joint_data.position[3], joint_data.position[4], joint_data.position[5]);
}
```

## Nächste Schritte

1. **Motor-Controller Integration**: Implementieren Sie die spezifische Kommunikation mit Ihren Motor-Controllern in `process_joint_commands()`
2. **PID-Regelung**: Fügen Sie PID-Controller für präzise Positionsregelung hinzu
3. **Sicherheit**: Implementieren Sie Limit-Checks und Emergency-Stop-Funktionalität
4. **Feedback**: Senden Sie Motor-Feedback zurück an ROS2 über `/topic_based_custom_joint_states`

## Wichtige Hinweise

- Alle Winkel sind in **Radiant** (nicht Grad)
- Die Gelenkreihenfolge entspricht der in `ros2_controllers.yaml` definierten Reihenfolge
- Berücksichtigen Sie die Gear Ratios bei der Umrechnung in Motor-Positionen
- Beachten Sie die `inverted` Flags aus `ros2_control.xacro` für die Motorrichtung
