// File: arctos_esp32_topic_based.ino
// Framework: Arduino + micro_ros_arduino
// Board: ESP32 (z.B. ESP32-WROOM)
// Libraries: micro_ros_arduino
// Purpose: Receive ROS 2 JointState commands, interpolate at 1 kHz, generate STEP/DIR for 6 axes, publish JointState feedback.

#include <Arduino.h>
#include <micro_ros_arduino.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <sensor_msgs/msg/joint_state.h>
#include <std_msgs/msg/bool.h>

#include <math.h>
#include <string.h>
#include <rosidl_runtime_c/string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define AXIS_COUNT 6

// Transport auswählen
#define USE_WIFI 0
#if USE_WIFI
  // TODO: anpassen
  #define WIFI_SSID     "YOUR_WIFI_SSID"
  #define WIFI_PASS     "YOUR_WIFI_PASS"
  #define AGENT_IP      "192.168.1.100"
  #define AGENT_PORT    8888
#else
  // Serial über USB/UART
  #define SERIAL_BAUD   115200
#endif

// Topics (müssen zu deiner Xacro/Launch passen)
static const char* JOINT_COMMANDS_TOPIC = "/topic_based_joint_commands";
static const char* JOINT_STATES_TOPIC   = "/topic_based_joint_states";
static const char* ESTOP_TOPIC          = "/arctos/estop";

// Joint-Namen (Reihenfolge muss zu URDF/Controller passen)
static const char* JOINT_NAMES[AXIS_COUNT] = {
  "X_joint", "Y_joint", "Z_joint", "A_joint", "B_joint", "C_joint"
};

// GPIO Mapping (TODO: pins anpassen)
const int STEP_PIN[AXIS_COUNT] = { 34, 32, 25, 27, 12, 23 };
const int DIR_PIN[AXIS_COUNT]  = { 35, 33, 26,  14,  13, 22 };
// Enable-Pins (LOW = enable üblich bei A4988/TMC). Passe bei Bedarf an.
// Hinweis: X/Y/Z teilen sich GPIO16, A/B/C teilen sich GPIO17
const int ENA_PIN[AXIS_COUNT]  = { 19, 19, 19, 18, 18, 18 };

// LEDC Kanäle (0..15 verfügbar), 6 Achsen → 6 Kanäle
const int LEDC_CH[AXIS_COUNT]  = { 0, 1, 2, 3, 4, 5 };
const int LEDC_TIMER_BIT = 8;     // Duty-Bit-Tiefe (8 genügt)
const int LEDC_BASE_FREQ = 1000;  // Start-Frequenz; wird dynamisch angepasst

// Kinematik-/Antriebs-Parameter (TODO: anpassen)
const float STEPS_PER_REV[AXIS_COUNT] = { 200, 200, 200, 200, 200, 200 };
const float MICROSTEPPING[AXIS_COUNT] = { 16,  16,  16,  16,  16,  16  };
const float GEAR_RATIO[AXIS_COUNT]    = { 13.5, 150.0, 150.0, 48.0, 27.3375, 10.0 };
const bool  DIR_INVERT[AXIS_COUNT]    = { false, true, false, true, true, true };

// Limits (rad/s, rad/s^2) – initial konservativ, später tunen
float MAX_VEL[AXIS_COUNT]   = { 1.5, 0.8, 0.8, 1.2, 1.2, 2.5 };
float MAX_ACCEL[AXIS_COUNT] = { 5.0, 3.0, 3.0, 4.0, 4.0, 8.0 };

// Abgeleitet
float STEPS_PER_RAD[AXIS_COUNT];

// Interner Zustand
volatile bool estopped = false;

// Zielvorgaben aus ROS (Position in rad)
volatile float goal_pos[AXIS_COUNT] = {0};
volatile bool  goal_valid = false;

// Trajektorienzustand (1 kHz)
volatile float act_pos[AXIS_COUNT] = {0};     // rad
volatile float act_vel[AXIS_COUNT] = {0};     // rad/s
volatile float cmd_vel_limit[AXIS_COUNT];     // rad/s (kann zur Laufzeit angepasst)

// Aktuelle STEP-Frequenzen (Hz)
volatile float step_freq_curr[AXIS_COUNT] = {0};
volatile float step_freq_target[AXIS_COUNT] = {0};

// ROS
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rcl_subscription_t sub_cmd;
rcl_subscription_t sub_estop;
rcl_publisher_t pub_state;
rcl_timer_t pub_timer;
rclc_executor_t executor;

// Nachrichten
sensor_msgs__msg__JointState msg_cmd;
sensor_msgs__msg__JointState msg_state;
std_msgs__msg__Bool msg_estop;

// Arbeitsspeicher für JointState Arrays
char* name_data[AXIS_COUNT];
rosidl_runtime_c__String name_strings[AXIS_COUNT];
double position_data[AXIS_COUNT];
double velocity_data[AXIS_COUNT];
double effort_data[AXIS_COUNT]; // ungenutzt

double cmd_position_in[AXIS_COUNT]; // für Callback

// Hilfsfunktionen
void set_enable_all(bool enable)
{
  for (int i = 0; i < AXIS_COUNT; ++i) {
    if (ENA_PIN[i] >= 0) {
      pinMode(ENA_PIN[i], OUTPUT);
      digitalWrite(ENA_PIN[i], enable ? LOW : HIGH); // ggf. invertiert anpassen
    }
  }
}

void ledc_apply_freq(int ch, int pin, float freq_hz)
{
  if (freq_hz <= 0.5f) {
    // aus
    ledcWrite(ch, 0);
    return;
  }
  // Begrenzen
  float f = constrain(freq_hz, 1.0f, 100000.0f);
  // duty = 50%
  uint32_t duty = (1u << LEDC_TIMER_BIT) / 2u;

  // Hinweis: Häufiges ledcSetup kann Jitter erzeugen; für 1 kHz ok
  ledcSetup(ch, f, LEDC_TIMER_BIT);
  ledcAttachPin(pin, ch);
  ledcWrite(ch, duty);
}

inline float signf(float x) { return (x > 0) - (x < 0); }

// 1 kHz Steuer-Task: trapezförmige Fahrt zum Ziel mit Accel/Vel Limits
void control_task(void *arg)
{
  const TickType_t period = pdMS_TO_TICKS(1);
  TickType_t last = xTaskGetTickCount();

  while (true) {
    vTaskDelayUntil(&last, period);
    if (estopped) {
      // Alles stoppen
      for (int i = 0; i < AXIS_COUNT; ++i) {
        act_vel[i] = 0;
        step_freq_target[i] = 0;
        ledc_apply_freq(LEDC_CH[i], STEP_PIN[i], 0);
      }
      continue;
    }

    // 1 ms
    const float dt = 0.001f;

    for (int i = 0; i < AXIS_COUNT; ++i) {
      float d = goal_pos[i] - act_pos[i]; // verbleibende Strecke
      float vmax = MAX_VEL[i];
      float acc  = MAX_ACCEL[i];

      // Bremsweg
      float decel_dist = (act_vel[i]*act_vel[i]) / (2.0f * acc + 1e-9f);

      // Sollgeschwindigkeit
      float v_target = 0.0f;
      if (fabsf(d) > 1e-6f) {
        float dir = signf(d);
        if (fabsf(d) <= decel_dist) {
          // Abbremsen
          float nv = fabsf(act_vel[i]) - acc * dt;
          if (nv < 0.0f) nv = 0.0f;
          v_target = dir * nv;
        } else {
          // Beschleunigen Richtung Ziel
          v_target = act_vel[i] + dir * acc * dt;
          // Clamp
          if (fabsf(v_target) > vmax) v_target = dir * vmax;
        }
      } else {
        // Am Ziel
        v_target = 0.0f;
      }

      // Update Zustand
      float v_next = v_target;
      float v_avg = 0.5f * (act_vel[i] + v_next);
      act_pos[i] += v_avg * dt;
      act_vel[i]  = v_next;

      // Richtung + STEP-Frequenz
      bool dir_is_pos = (act_vel[i] >= 0.0f);
      bool hw_dir = dir_is_pos ^ DIR_INVERT[i];
      digitalWrite(DIR_PIN[i], hw_dir ? HIGH : LOW);

      float freq = fabsf(act_vel[i]) * STEPS_PER_RAD[i];
      step_freq_target[i] = freq;

      // Frequenz anwenden
      ledc_apply_freq(LEDC_CH[i], STEP_PIN[i], freq);
    }
  }
}

// ROS Callbacks
void cmd_callback(const void* msgin)
{
  const sensor_msgs__msg__JointState* m = (const sensor_msgs__msg__JointState*)msgin;

  // Map nach Name, wenn gegeben; sonst direkte Reihenfolge
  if (m->name.size == AXIS_COUNT && m->position.size == AXIS_COUNT) {
    for (size_t i = 0; i < m->name.size; ++i) {
      const char* nm = m->name.data[i].data;
      int idx = -1;
      for (int j = 0; j < AXIS_COUNT; ++j) {
        if (strcmp(nm, JOINT_NAMES[j]) == 0) { idx = j; break; }
      }
      if (idx >= 0) {
        goal_pos[idx] = (float)m->position.data[i];
      }
    }
  } else {
    // Fallback: Index-basiert
    size_t n = min((size_t)AXIS_COUNT, m->position.size);
    for (size_t i = 0; i < n; ++i) {
      goal_pos[i] = (float)m->position.data[i];
    }
  }
  goal_valid = true;
}

void estop_callback(const void* msgin)
{
  const std_msgs__msg__Bool* m = (const std_msgs__msg__Bool*)msgin;
  estopped = m->data;
  if (estopped) {
    set_enable_all(false);
  } else {
    set_enable_all(true);
  }
}

void pub_timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  (void)timer; (void)last_call_time;
  // Fülle JointState
  msg_state.header.stamp.sec  = (int32_t)(millis()/1000);
  msg_state.header.stamp.nanosec = (uint32_t)((millis()%1000)*1000000);

  // Position/Velocity aus aktuellem Zustand
  for (int i = 0; i < AXIS_COUNT; ++i) {
    position_data[i] = (double)act_pos[i];
    velocity_data[i] = (double)act_vel[i];
  }
  msg_state.position.data = position_data;
  msg_state.position.size = AXIS_COUNT;
  msg_state.velocity.data = velocity_data;
  msg_state.velocity.size = AXIS_COUNT;
  // effort leer

  rcl_publish(&pub_state, &msg_state, NULL);
}

// Setup micro-ROS Entities
bool create_entities()
{
  allocator = rcl_get_default_allocator();

  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "arctos_esp32", "", &support);

  rclc_subscription_init_default(
    &sub_cmd, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
    JOINT_COMMANDS_TOPIC);

  rclc_subscription_init_default(
    &sub_estop, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    ESTOP_TOPIC);

  rclc_publisher_init_default(
    &pub_state, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
    JOINT_STATES_TOPIC);

  // Timer ~50 Hz für State Publish
  const unsigned int pub_period_ms = 20;
  rclc_timer_init_default(&pub_timer, &support, RCL_MS_TO_NS(pub_period_ms), pub_timer_callback);

  // Executor
  rclc_executor_init(&executor, &support.context, 3, &allocator);
  rclc_executor_add_subscription(&executor, &sub_cmd, &msg_cmd, &cmd_callback, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &sub_estop, &msg_estop, &estop_callback, ON_NEW_DATA);
  rclc_executor_add_timer(&executor, &pub_timer);

  return true;
}

void setup_jointstate_messages()
{
  // Zero-init messages (wichtig für micro-ROS)
  memset(&msg_state, 0, sizeof(msg_state));
  memset(&msg_cmd, 0, sizeof(msg_cmd));
  memset(&msg_estop, 0, sizeof(msg_estop));

  // Name-Strings vorbereiten
  msg_state.name.capacity = AXIS_COUNT;
  msg_state.name.size = AXIS_COUNT;
  msg_state.name.data = name_strings;
  for (int i = 0; i < AXIS_COUNT; ++i) {
    name_data[i] = (char*)JOINT_NAMES[i];
    msg_state.name.data[i].data = name_data[i];
    msg_state.name.data[i].capacity = strlen(JOINT_NAMES[i]) + 1;
    msg_state.name.data[i].size = strlen(JOINT_NAMES[i]);
  }

  msg_state.position.capacity = AXIS_COUNT;
  msg_state.velocity.capacity = AXIS_COUNT;
  msg_state.effort.capacity   = AXIS_COUNT;
  // pointer setzen in pub_timer_callback vor publish
}

void setup_pins_and_ledc()
{
  for (int i = 0; i < AXIS_COUNT; ++i) {
    pinMode(DIR_PIN[i], OUTPUT);
    pinMode(STEP_PIN[i], OUTPUT);
    if (ENA_PIN[i] >= 0) pinMode(ENA_PIN[i], OUTPUT);

    // LEDC init
    ledcSetup(LEDC_CH[i], LEDC_BASE_FREQ, LEDC_TIMER_BIT);
    ledcAttachPin(STEP_PIN[i], LEDC_CH[i]);
    ledcWrite(LEDC_CH[i], 0); // aus
  }
  set_enable_all(true);
}

void compute_derived()
{
  for (int i = 0; i < AXIS_COUNT; ++i) {
    STEPS_PER_RAD[i] = (STEPS_PER_REV[i] * MICROSTEPPING[i] * GEAR_RATIO[i]) / (2.0f * (float)M_PI);
    cmd_vel_limit[i] = MAX_VEL[i];
  }
}

TaskHandle_t controlTaskHandle = nullptr;

void setup()
{
#if USE_WIFI
  set_microros_wifi_transports(WIFI_SSID, WIFI_PASS, AGENT_IP, AGENT_PORT);
#else
  Serial.begin(SERIAL_BAUD);
  set_microros_serial_transports(Serial);
#endif

  // Physik
  compute_derived();
  setup_pins_and_ledc();

  // micro-ROS Messages init
  setup_jointstate_messages();

  // Entities
  create_entities();

  // 1 kHz Control Task (hohe Prio); Core 1
  xTaskCreatePinnedToCore(control_task, "ctrl1khz", 4096, NULL, configMAX_PRIORITIES - 2, &controlTaskHandle, 1);
}

void loop()
{
  // micro-ROS Executor laufen lassen
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));
  delay(1);
}
