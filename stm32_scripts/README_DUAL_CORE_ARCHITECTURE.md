# STM32H755 Dual-Core Architektur für Step/Dir Motorsteuerung

## Übersicht

Diese Implementierung nutzt die Dual-Core-Architektur des STM32H755ZiQ für eine hochperformante Schrittmotorsteuerung:

- **M7 Core (Cortex-M7 @ 480 MHz)**: ROS2 Kommunikation und Kinematik-Berechnungen
- **M4 Core (Cortex-M4 @ 240 MHz)**: Echtzeit Step/Dir Puls-Generierung

## Architektur-Übersicht

```
┌─────────────────────────────────────────────────────────────────┐
│                         M7 Core (CM7)                            │
├─────────────────────────────────────────────────────────────────┤
│  • Micro-ROS (UART3 Transport)                                  │
│  • Subscriber: /topic_based_joint_commands (JointState)         │
│  • Publisher: /stm32_m7_status (Int32)                          │
│                                                                   │
│  Verarbeitung:                                                   │
│  1. Empfang Gelenkwinkel (Radiant)                              │
│  2. Umrechnung: Radiant → Motor-Umdrehungen (Gear Ratio)        │
│  3. Umrechnung: Umdrehungen → Steps (Steps/Rev)                 │
│  4. Schreiben in Shared Memory (D3 SRAM)                        │
└─────────────────────────────────────────────────────────────────┘
                              ↓
                    ┌─────────────────┐
                    │  Shared Memory  │
                    │  (D3 SRAM)      │
                    │  0x38000000     │
                    │                 │
                    │  • Target Steps │
                    │  • Current Steps│
                    │  • Speed/Accel  │
                    │  • Enable Flags │
                    │  • HSEM Lock    │
                    └─────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────────┐
│                         M4 Core (CM4)                            │
├─────────────────────────────────────────────────────────────────┤
│  • Step/Dir Puls-Generierung (Hardware Timer)                   │
│  • Trapezoid Motion Profile                                     │
│  • 6-Achsen simultane Steuerung                                 │
│                                                                   │
│  Verarbeitung:                                                   │
│  1. Lesen Target Steps aus Shared Memory                        │
│  2. Berechnung Bewegungsprofil (Trapezoid)                      │
│  3. Echtzeit Step/Dir Puls-Generierung                          │
│  4. Aktualisierung Current Steps                                │
└─────────────────────────────────────────────────────────────────┘
                              ↓
                    ┌─────────────────┐
                    │  GPIO Outputs   │
                    │                 │
                    │  6x STEP Pins   │
                    │  6x DIR Pins    │
                    └─────────────────┘
                              ↓
                    ┌─────────────────┐
                    │ Stepper Drivers │
                    │  (TB6600, etc.) │
                    └─────────────────┘
```

## Shared Memory Struktur

### Speicher-Layout (D3 SRAM @ 0x38000000)

```c
typedef struct {
  // Soll-Positionen in Motor-Steps (von M7 geschrieben)
  int32_t target_steps[6];        // Offset: 0x00 (24 bytes)
  
  // Ist-Positionen in Motor-Steps (von M4 geschrieben)
  int32_t current_steps[6];       // Offset: 0x18 (24 bytes)
  
  // Maximale Geschwindigkeit in Steps/Sekunde
  uint32_t max_speed[6];          // Offset: 0x30 (24 bytes)
  
  // Beschleunigung in Steps/s²
  uint32_t acceleration[6];       // Offset: 0x48 (24 bytes)
  
  // Motor Enable Flags (Bit 0-5)
  uint8_t motor_enable;           // Offset: 0x60 (1 byte)
  
  // Emergency Stop Flag
  volatile uint8_t emergency_stop; // Offset: 0x61 (1 byte)
  
  // Daten-Gültig Flag (M7→M4)
  volatile uint8_t data_valid;    // Offset: 0x62 (1 byte)
  
  // M4 Bereit Flag (M4→M7)
  volatile uint8_t m4_ready;      // Offset: 0x63 (1 byte)
  
  // Gear Ratios (Referenz)
  float gear_ratios[6];           // Offset: 0x64 (24 bytes)
  
  // Steps pro Umdrehung
  uint32_t steps_per_rev[6];      // Offset: 0x7C (24 bytes)
  
  // Richtungs-Invertierung (Bit 0-5)
  uint8_t direction_inverted;     // Offset: 0x94 (1 byte)
  
} SharedMotorData_t;  // Total: ~149 bytes (aligned to 32 bytes = 160 bytes)
```

## Synchronisation mit HSEM (Hardware Semaphore)

### M7 Core - Schreiben in Shared Memory

```c
// Take HSEM
if (HAL_HSEM_FastTake(HSEM_ID_0) == HAL_OK) {
  
  // Update shared memory
  for (int i = 0; i < 6; i++) {
    shared_motor_data.target_steps[i] = calculated_steps[i];
  }
  shared_motor_data.data_valid = 1;
  
  // Release HSEM
  HAL_HSEM_Release(HSEM_ID_0, 0);
}
```

### M4 Core - Lesen aus Shared Memory

```c
// Take HSEM
if (HAL_HSEM_FastTake(HSEM_ID_0) == HAL_OK) {
  
  // Read shared memory
  if (shared_motor_data.data_valid) {
    for (int i = 0; i < 6; i++) {
      motor_states[i].target_position = shared_motor_data.target_steps[i];
    }
    shared_motor_data.data_valid = 0;
  }
  
  // Write current positions back
  for (int i = 0; i < 6; i++) {
    shared_motor_data.current_steps[i] = motor_states[i].current_position;
  }
  
  // Release HSEM
  HAL_HSEM_Release(HSEM_ID_0, 0);
}
```

## M7 Core - ROS2 Integration

### Datenfluss

1. **ROS2 Topic empfangen**: `/topic_based_joint_commands` (sensor_msgs/JointState)
   ```
   position: [X, Y, Z, A, B, C] in Radiant
   velocity: [vX, vY, vZ, vA, vB, vC] in rad/s
   ```

2. **Umrechnung Radiant → Steps**:
   ```c
   motor_revolutions = (position_rad / (2π)) * gear_ratio
   steps = motor_revolutions * steps_per_rev
   ```

3. **Gear Ratios** (aus ros2_control.xacro):
   - X_joint: 13.5
   - Y_joint: 150.0
   - Z_joint: 150.0
   - A_joint: 48.0
   - B_joint: 27.3375
   - C_joint: 10.0

4. **Beispiel-Berechnung** (X_joint):
   ```
   Input: 1.57 rad (90°)
   motor_rev = (1.57 / 6.283) * 13.5 = 3.375 Umdrehungen
   steps = 3.375 * 3200 = 10800 Steps (bei 16x Microstepping)
   ```

## M4 Core - Step/Dir Engine

### Trapezoid Motion Profile

```
Speed
  ^
  │     ┌─────────┐  Max Speed
  │    ╱           ╲
  │   ╱             ╲
  │  ╱               ╲
  │ ╱                 ╲
  └─────────────────────> Steps
    Accel  Constant  Decel
```

### Phasen

1. **Beschleunigung**: `v = √(2 * a * s)`
2. **Konstante Geschwindigkeit**: `v = v_max`
3. **Verzögerung**: `v = √(2 * a * s_remaining)`

### Timer-Interrupt (TIM6)

```c
void TIM6_DAC_IRQHandler(void)
{
  for (int i = 0; i < 6; i++) {
    if (motor_moving[i]) {
      // Berechne aktuelle Geschwindigkeit basierend auf Profil
      calculate_speed(i);
      
      // Generiere Step-Puls
      generate_step_pulse(i);
      
      // Update Position
      current_position[i] += direction[i];
    }
  }
}
```

## Hardware-Konfiguration

### GPIO Pin-Zuordnung (Beispiel)

| Motor | Gelenk  | STEP Pin | DIR Pin  |
|-------|---------|----------|----------|
| 0     | X_joint | PA0      | PA1      |
| 1     | Y_joint | PA2      | PA3      |
| 2     | Z_joint | PA4      | PA5      |
| 3     | A_joint | PA6      | PA7      |
| 4     | B_joint | PB0      | PB1      |
| 5     | C_joint | PB2      | PB3      |

**Hinweis**: Passen Sie diese Pins an Ihre Hardware an!

### Stepper Driver Anschluss (z.B. TB6600)

```
STM32 STEP Pin ──→ PUL+ (Driver)
STM32 GND      ──→ PUL- (Driver)
STM32 DIR Pin  ──→ DIR+ (Driver)
STM32 GND      ──→ DIR- (Driver)
```

### Microstepping-Konfiguration

Standard: **16x Microstepping**
- 200 Steps/Rev (Vollschritt) × 16 = **3200 Steps/Rev**

Anpassbar in `init_shared_motor_data()`:
```c
uint32_t microsteps = 16;  // 1, 2, 4, 8, 16, 32
shared_motor_data.steps_per_rev[i] = 200 * microsteps;
```

## Linker-Script Konfiguration

### M7 Core Linker Script

Fügen Sie folgende Sektion hinzu:

```ld
MEMORY
{
  /* ... existing regions ... */
  SHARED_RAM (xrw) : ORIGIN = 0x38000000, LENGTH = 64K
}

SECTIONS
{
  .shared_data (NOLOAD) :
  {
    . = ALIGN(32);
    *(.shared_data)
    . = ALIGN(32);
  } >SHARED_RAM
}
```

### M4 Core Linker Script

Gleiche SHARED_RAM Region verwenden!

## Build-Konfiguration

### STM32CubeIDE

1. **M7 Projekt**:
   - Linker Script: Fügen Sie `.shared_data` Section hinzu
   - Compiler Flags: `-DCORE_CM7`

2. **M4 Projekt**:
   - Linker Script: Gleiche SHARED_RAM Region
   - Compiler Flags: `-DCORE_CM4`

3. **Dual-Core Boot**:
   - M7 startet zuerst
   - M7 initialisiert Shared Memory
   - M7 startet M4 via HSEM

## Performance-Parameter

### Empfohlene Werte

| Parameter | Wert | Einheit | Beschreibung |
|-----------|------|---------|--------------|
| Max Speed | 10000 | steps/s | Maximale Schrittfrequenz |
| Acceleration | 5000 | steps/s² | Beschleunigung |
| Timer Freq | 10000 | Hz | TIM6 Update-Rate |
| ROS2 Update | 100 | Hz | Control Loop Rate |

### Anpassung für Ihre Motoren

```c
// In init_shared_motor_data()
shared_motor_data.max_speed[0] = 20000;      // Schnellerer X-Motor
shared_motor_data.acceleration[0] = 10000;   // Höhere Beschleunigung
```

## Debugging

### M7 Status überwachen

```bash
ros2 topic echo /stm32_m7_status
```

### Shared Memory inspizieren (via Debugger)

```
(gdb) x/40xw 0x38000000
```

### M4 Step-Generierung testen

```c
// In M4 main loop
if (motor_states[0].moving) {
  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);  // LED blinkt bei Bewegung
}
```

## Sicherheits-Features

### Emergency Stop

```c
// M7: Trigger Emergency Stop
shared_motor_data.emergency_stop = 1;

// M4: Alle Motoren sofort stoppen
if (shared_motor_data.emergency_stop) {
  for (int i = 0; i < 6; i++) {
    motor_states[i].moving = false;
  }
}
```

### Timeout-Handling

```c
// M7: Prüft auf veraltete ROS2-Daten
if ((HAL_GetTick() - joint_data.last_update_time) > 1000) {
  shared_motor_data.emergency_stop = 1;  // Stop nach 1s Timeout
}
```

### Motor Enable/Disable

```c
// Einzelne Motoren deaktivieren
shared_motor_data.motor_enable &= ~(1 << motor_idx);  // Disable
shared_motor_data.motor_enable |= (1 << motor_idx);   // Enable
```

## Nächste Schritte

### 1. M4 Projekt erstellen
- Neues STM32CubeIDE Projekt für M4 Core
- `m4_step_dir_engine.c` integrieren
- GPIO Pins konfigurieren
- Timer TIM6 konfigurieren

### 2. Linker Scripts anpassen
- Shared Memory Section in beiden Projekten
- Cache-Kohärenz sicherstellen (32-byte alignment)

### 3. Hardware-Tests
- Step/Dir Signale mit Oszilloskop prüfen
- Pulsbreite und Timing verifizieren
- Microstepping-Einstellungen am Driver prüfen

### 4. Integration
- M7 und M4 zusammen flashen
- HSEM-Synchronisation testen
- End-to-End Test: ROS2 → M7 → M4 → Motoren

### 5. Optimierung
- Timer-Frequenz anpassen
- Motion Profile tunen
- Jerk-Limiting implementieren (optional)

## Erweiterte Features (Optional)

### Homing-Sequenz
```c
// Endschalter-Pins konfigurieren
// Homing-Routine in M4 implementieren
```

### Position Feedback
```c
// Encoder-Input für Closed-Loop
// Vergleich target_steps vs. encoder_position
```

### Dynamische Geschwindigkeit
```c
// Geschwindigkeit basierend auf ROS2 velocity-Feld
double velocity_rad_s = joint_data.velocity[i];
uint32_t speed_steps_s = calculate_speed_from_velocity(velocity_rad_s);
```

## Troubleshooting

### Problem: M4 empfängt keine Daten
- Prüfen: `shared_motor_data.m4_ready == 1`
- Prüfen: HSEM korrekt initialisiert
- Prüfen: Linker Script `.shared_data` Section

### Problem: Unregelmäßige Step-Pulse
- Timer-Frequenz zu niedrig
- Interrupt-Priorität zu niedrig
- Andere Interrupts blockieren TIM6

### Problem: Motoren bewegen sich falsch
- Direction Inversion prüfen
- Gear Ratio verifizieren
- Steps/Rev Konfiguration prüfen

## Referenzen

- STM32H755 Reference Manual (RM0399)
- STM32H7 Dual-Core Programming Manual (PM0253)
- Micro-ROS Documentation
- ROS2 Control Documentation
