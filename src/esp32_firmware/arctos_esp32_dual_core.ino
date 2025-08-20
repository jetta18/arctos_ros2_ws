// ============================================================================
//  Arctos ESP32 Firmware (Timer-ISR Step Engine + micro-ROS in loop())
//  - Step/DIR Pulser per Hardware-Timer (50 µs Takt) -> deterministisch
//  - micro-ROS empfängt JointState und setzt nur Sollwerte
//  - Vollständige Vorallokation des JointState-Subscribers (inkl. header.frame_id)
//  - Diagnose-LED blinkt bei jedem empfangenen JointState
//  - WICHTIG: ISR nutzt IRAM-sichere, direkte GPIO-Register (kein digitalWrite!)
// ============================================================================

#include <Arduino.h>
#include <micro_ros_arduino.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/joint_state.h>
#include <std_msgs/msg/string.h>
#include <rosidl_runtime_c/string.h>
#include <rmw_microros/rmw_microros.h>

#include <math.h>
#include <string.h>

// ---- ESP32 Low-level GPIO (für ISR) ----
#include "driver/gpio.h"
#include "soc/gpio_struct.h"
#include "soc/gpio_reg.h"

// ---- Reset-Reason API (Fix) ----
#include <esp_system.h>   // esp_reset_reason()

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// ----------------------------- Konfiguration -----------------------------

#define AXIS_COUNT 6
#define SERIAL_BAUD 115200

// Diagnose-LED (auf vielen Dev-Modulen GPIO2). Auf -1 setzen zum Deaktivieren.
#define DIAG_LED 2

// Optionaler Boot-Testmove (1=an, 0=aus)
#define BOOT_TEST_MOVE 0

// ROS Topics
static const char* JOINT_COMMANDS_TOPIC = "/topic_based_joint_commands";
static const char* JOINT_STATES_TOPIC   = "/topic_based_joint_states";
static const char* DEBUG_TOPIC          = "/arctos_debug";

// Joint-Namen
static const char* JOINT_NAMES[AXIS_COUNT] = {
  "X_joint","Y_joint","Z_joint","A_joint","B_joint","C_joint"
};

// GPIOs
const int STEP_PIN[AXIS_COUNT] = { 23, 32, 25, 27, 12, 21 };
const int DIR_PIN [AXIS_COUNT] = { 22, 33, 26, 14, 13,  5 };
const int ENA_PIN [AXIS_COUNT] = { 19, 19, 19, 18, 18, 18 }; // LOW=Enable

// Mechanik
const float STEPS_PER_REV [AXIS_COUNT] = { 200,   200,   200,   200,    200,   200 };
const float MICROSTEPPING [AXIS_COUNT] = { 16,     16,    16,    16,     16,    16 };
const float GEAR_RATIO    [AXIS_COUNT] = { 13.5, 150.0, 150.0,  48.0, 27.3375, 10.0 };
const bool  DIR_INVERT    [AXIS_COUNT] = { false, true,  false,  true,   true,   true };

// Bewegungslimits (keine Drossel ergänzt)
const float MAX_SPEED_RAD_S     [AXIS_COUNT] = { 2.0f, 1.0f, 1.0f, 2.5f, 2.5f, 4.0f };
const float DEFAULT_SPEED_RAD_S [AXIS_COUNT] = { 0.8f, 0.5f, 0.5f, 1.0f, 1.0f, 2.0f };

// Step-Timing
static const int  MIN_PULSE_WIDTH_US = 10; // sichere 10 µs
static const int  TIMER_TICK_US      = 50; // ISR alle 50 µs -> 20 kHz
static const int  PULSE_TICKS        = (MIN_PULSE_WIDTH_US + TIMER_TICK_US - 1) / TIMER_TICK_US; // >=1
static const int  MIN_TICKS_PER_STEP = 1;  // max ~20 kSteps/s

// ----------------------- Abgeleitete Größen / Zustände --------------------

float STEPS_PER_RAD[AXIS_COUNT];
uint32_t DEFAULT_TICKS_PER_STEP[AXIS_COUNT];

typedef struct {
  volatile long  current_pos;
  volatile long  target_pos;
  volatile bool  moving;
  volatile uint32_t ticks_per_step;
  volatile int32_t  tick_counter;
  volatile uint8_t  pulse_ticks;
  volatile bool     dir_state;
} axis_rt_t;

axis_rt_t axis_rt[AXIS_COUNT];

typedef struct {
  volatile long  target_pos;
  volatile bool  new_cmd;
} axis_cmd_t;

axis_cmd_t axis_cmd[AXIS_COUNT];

typedef struct {
  volatile long  current_pos;
  /* speed removed */
  volatile bool  moving;
} axis_pub_t;

axis_pub_t axis_pub[AXIS_COUNT];

// ISR/Debug Instrumentation
volatile uint32_t isr_step_count[AXIS_COUNT];
volatile uint32_t isr_dir_change_count[AXIS_COUNT];
volatile uint32_t isr_cmd_applied_count[AXIS_COUNT];

// ------------------------------ micro-ROS ---------------------------------

rclc_support_t   support;
rcl_node_t       node;
rcl_allocator_t  allocator;
rclc_executor_t  executor;
rcl_subscription_t sub_cmd;
rcl_publisher_t    pub_state;
rcl_publisher_t    pub_debug;
rcl_timer_t        pub_timer;

sensor_msgs__msg__JointState msg_cmd;
sensor_msgs__msg__JointState msg_state_pub;
std_msgs__msg__String        msg_debug;

double                   pub_positions[AXIS_COUNT];
double                   pub_velocities[AXIS_COUNT];
rosidl_runtime_c__String pub_names[AXIS_COUNT];

double                   sub_positions[AXIS_COUNT];
double                   sub_velocities[AXIS_COUNT];
rosidl_runtime_c__String sub_names[AXIS_COUNT];
char                     sub_name_storage[AXIS_COUNT][16];
char                     sub_frame_id_buf[32];

// ------------------------------ Timer-ISR ---------------------------------

hw_timer_t* stepTimer = nullptr;

// IRAM-sichere, ultraschnelle GPIO-Helper
static inline __attribute__((always_inline)) IRAM_ATTR void gpio_set_fast(int pin) {
  if (pin < 32) GPIO.out_w1ts = (1UL << pin);
  else          GPIO.out1_w1ts.val = (1UL << (pin - 32));
}
static inline __attribute__((always_inline)) IRAM_ATTR void gpio_clr_fast(int pin) {
  if (pin < 32) GPIO.out_w1tc = (1UL << pin);
  else          GPIO.out1_w1tc.val = (1UL << (pin - 32));
}
static inline __attribute__((always_inline)) IRAM_ATTR void gpio_dir_fast(int pin, bool level_high) {
  if (level_high) gpio_set_fast(pin); else gpio_clr_fast(pin);
}

inline uint32_t speed_to_ticks(float steps_per_s) {
  if (steps_per_s <= 0.0f) return 0x7FFFFFFF;
  const float timer_hz = 1e6f / (float)TIMER_TICK_US; // 20 kHz
  float ticks = timer_hz / steps_per_s;
  if (ticks < (float)MIN_TICKS_PER_STEP) ticks = (float)MIN_TICKS_PER_STEP;
  if (ticks > (float)0x7FFFFFFF) ticks = (float)0x7FFFFFFF;
  return (uint32_t)lroundf(ticks);
}

void IRAM_ATTR onStepTimer() {
  for (int i = 0; i < AXIS_COUNT; ++i) {
    // 1) Pending commands übernehmen
    if (axis_cmd[i].new_cmd) {
      axis_rt[i].target_pos     = axis_cmd[i].target_pos;
      axis_rt[i].ticks_per_step = DEFAULT_TICKS_PER_STEP[i];
      axis_rt[i].moving         = (axis_rt[i].target_pos != axis_rt[i].current_pos);
      // Do not increase latency: only reset if expired or longer than new interval
      if (axis_rt[i].tick_counter <= 0 || axis_rt[i].tick_counter > axis_rt[i].ticks_per_step) {
        axis_rt[i].tick_counter = axis_rt[i].ticks_per_step;
      }
      axis_cmd[i].new_cmd       = false;
      isr_cmd_applied_count[i]++;
    }

    // 2) STEP High-Phase beenden
    if (axis_rt[i].pulse_ticks) {
      axis_rt[i].pulse_ticks--;
      if (axis_rt[i].pulse_ticks == 0) {
        gpio_clr_fast(STEP_PIN[i]); // Ende Puls
      }
    }

    // 3) Schritt fällig?
    if (axis_rt[i].moving) {
      axis_rt[i].tick_counter--;
      if (axis_rt[i].tick_counter <= 0) {
        bool dir_needed = (axis_rt[i].target_pos > axis_rt[i].current_pos);
        if (dir_needed != axis_rt[i].dir_state) {
          axis_rt[i].dir_state = dir_needed;
          bool out = dir_needed ^ DIR_INVERT[i];
          gpio_dir_fast(DIR_PIN[i], out);
          // Count dir changes and ensure setup time before next STEP by delaying one period
          isr_dir_change_count[i]++;
          axis_rt[i].tick_counter += axis_rt[i].ticks_per_step; // wait one more interval
          continue; // do not emit STEP in the same tick as DIR change
        }

        // STEP-Puls setzen
        gpio_set_fast(STEP_PIN[i]);
        axis_rt[i].pulse_ticks = (PULSE_TICKS > 0) ? PULSE_TICKS : 1;
        isr_step_count[i]++;

        // Position aktualisieren
        axis_rt[i].current_pos += dir_needed ? +1 : -1;

        // nächsten Termin
        axis_rt[i].tick_counter += axis_rt[i].ticks_per_step;

        // Ziel?
        if (axis_rt[i].current_pos == axis_rt[i].target_pos) {
          axis_rt[i].moving = false;
        }
      }
    }

    // 4) Werte für Publisher (näherungsweise)
    axis_pub[i].current_pos = axis_rt[i].current_pos;
    axis_pub[i].moving      = axis_rt[i].moving;
    /* speed removed */
  }
}

// ------------------------------ Utilities ---------------------------------

void set_motors_enabled(bool enable) {
  for (int i = 0; i < AXIS_COUNT; ++i) {
    if (ENA_PIN[i] >= 0) {
      digitalWrite(ENA_PIN[i], enable ? LOW : HIGH); // viele Treiber: LOW=Enable
    }
  }
}

long   rad_to_steps (int a, double rad)      { return (a<0||a>=AXIS_COUNT)?0:(long) llround(rad * STEPS_PER_RAD[a]); }
double steps_to_rad (int a, long steps)      { return (a<0||a>=AXIS_COUNT)?0.0: (double)steps / STEPS_PER_RAD[a]; }
// radps_to_sps removed

// ------------------------------ ROS Callbacks -----------------------------

static uint32_t cmd_count = 0;

void debug_publish(const char* s) {
  size_t n = strlen(s);
  if (n >= msg_debug.data.capacity) n = msg_debug.data.capacity - 1;
  memcpy(msg_debug.data.data, s, n);
  msg_debug.data.data[n] = 0;
  msg_debug.data.size = n;
  (void) rcl_publish(&pub_debug, &msg_debug, NULL); // Warnung beseitigt
}

void cmd_callback(const void* msgin) {
  if (DIAG_LED >= 0) { digitalWrite(DIAG_LED, HIGH); delayMicroseconds(40); digitalWrite(DIAG_LED, LOW); }

  const auto* m = (const sensor_msgs__msg__JointState*)msgin;
  const size_t n_pos  = m->position.size;
  const size_t n_name = m->name.size; // ignored for mapping
  if (n_pos == 0) return;

  ++cmd_count;

  auto apply = [&](int axis, size_t idx){
    double pos = (idx < n_pos) ? m->position.data[idx] : 0.0;
    long  tgt_steps = rad_to_steps(axis, pos);
    axis_cmd[axis].target_pos = tgt_steps;
    axis_cmd[axis].new_cmd = true;
  };

  size_t n = (n_pos < (size_t)AXIS_COUNT) ? n_pos : (size_t)AXIS_COUNT;
  for (size_t i = 0; i < n; ++i) apply((int)i, i);

  // immediate publish removed for simplicity; timer will publish at fixed rate

  char buf[160];
  snprintf(buf, sizeof(buf),
           "[cmd #%lu] pos=%u axis0_tgt=%ld",
           (unsigned long)cmd_count, (unsigned)n_pos,
           (long)axis_cmd[0].target_pos);
  debug_publish(buf);
}

void pub_timer_callback(rcl_timer_t*, int64_t) {
  uint64_t now_ns = rmw_uros_epoch_nanos();
  if (now_ns > 0) {
    msg_state_pub.header.stamp.sec     = (int32_t)(now_ns / 1000000000ULL);
    msg_state_pub.header.stamp.nanosec = (uint32_t)(now_ns % 1000000000ULL);
  } else {
    uint32_t ms = millis();
    msg_state_pub.header.stamp.sec     = (int32_t)(ms / 1000);
    msg_state_pub.header.stamp.nanosec = (uint32_t)((ms % 1000) * 1000000UL);
  }

  for (int i = 0; i < AXIS_COUNT; ++i) {
    // Mirror commanded (target) positions to published state
    pub_positions[i]  = steps_to_rad(i, axis_rt[i].target_pos);
    pub_velocities[i] = 0.0; // explicitly zero velocities
  }
  (void) rcl_publish(&pub_state, &msg_state_pub, NULL);

  char hb[160];
  snprintf(hb, sizeof(hb),
           "HB X:pos=%ld mov=%d steps=%lu dirchg=%lu cmds=%lu | Y:pos=%ld mov=%d",
           (long)axis_pub[0].current_pos, (int)axis_pub[0].moving,
           (unsigned long)isr_step_count[0], (unsigned long)isr_dir_change_count[0], (unsigned long)isr_cmd_applied_count[0],
           (long)axis_pub[1].current_pos, (int)axis_pub[1].moving);
  debug_publish(hb);
}

// --------------------------- ROS Entities/Buffer --------------------------

void setup_publisher_message() {
  msg_state_pub.name.data     = pub_names;
  msg_state_pub.name.capacity = AXIS_COUNT;
  msg_state_pub.name.size     = AXIS_COUNT;
  for (int i = 0; i < AXIS_COUNT; ++i) {
    pub_names[i].data     = (char*)JOINT_NAMES[i];
    pub_names[i].capacity = strlen(JOINT_NAMES[i]) + 1;
    pub_names[i].size     = strlen(JOINT_NAMES[i]);
  }
  msg_state_pub.position.data     = pub_positions;
  msg_state_pub.position.capacity = AXIS_COUNT;
  msg_state_pub.position.size     = AXIS_COUNT;

  msg_state_pub.velocity.data     = pub_velocities;
  msg_state_pub.velocity.capacity = AXIS_COUNT;
  msg_state_pub.velocity.size     = AXIS_COUNT;

  msg_state_pub.effort.data     = NULL;
  msg_state_pub.effort.capacity = 0;
  msg_state_pub.effort.size     = 0;

  msg_state_pub.header.frame_id.data     = NULL;
  msg_state_pub.header.frame_id.size     = 0;
  msg_state_pub.header.frame_id.capacity = 0;
}

void setup_subscriber_message() {
  msg_cmd.position.data     = sub_positions;
  msg_cmd.position.capacity = AXIS_COUNT;
  msg_cmd.position.size     = 0;

  msg_cmd.velocity.data     = sub_velocities;
  msg_cmd.velocity.capacity = AXIS_COUNT;
  msg_cmd.velocity.size     = 0;

  msg_cmd.name.data     = sub_names;
  msg_cmd.name.capacity = AXIS_COUNT;
  msg_cmd.name.size     = 0;
  for (int i = 0; i < AXIS_COUNT; ++i) {
    sub_names[i].data     = sub_name_storage[i];
    sub_names[i].capacity = sizeof(sub_name_storage[i]);
    sub_names[i].size     = 0;
  }

  msg_cmd.effort.data     = NULL;
  msg_cmd.effort.capacity = 0;
  msg_cmd.effort.size     = 0;

  msg_cmd.header.frame_id.data     = sub_frame_id_buf;
  msg_cmd.header.frame_id.capacity = sizeof(sub_frame_id_buf);
  msg_cmd.header.frame_id.size     = 0;
}

// Task-Handle für die micro-ROS-Schleife
TaskHandle_t microRosTaskHandle = NULL;

void microRosTask(void* pvParameters);

void create_entities() {
  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "arctos_esp32_firmware", "", &support);

  rclc_subscription_init_best_effort(
    &sub_cmd, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
    JOINT_COMMANDS_TOPIC
  );
  rclc_publisher_init_best_effort(
    &pub_state, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
    JOINT_STATES_TOPIC
  );
  rclc_publisher_init_best_effort(
    &pub_debug, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    DEBUG_TOPIC
  );

  const unsigned int pub_period_ms = 10; // 100 Hz aligns with controller
  rclc_timer_init_default(&pub_timer, &support, RCL_MS_TO_NS(pub_period_ms), pub_timer_callback);

  rclc_executor_init(&executor, &support.context, 3, &allocator);
  rclc_executor_add_subscription(&executor, &sub_cmd, &msg_cmd, &cmd_callback, ON_NEW_DATA);
  rclc_executor_add_timer(&executor, &pub_timer);
}

void destroy_entities() {
  // Reihenfolge: Executor -> Timer -> Pub/Sub -> Node -> Support
  rclc_executor_fini(&executor);
  rcl_timer_fini(&pub_timer);
  rcl_publisher_fini(&pub_debug, &node);
  rcl_publisher_fini(&pub_state, &node);
  rcl_subscription_fini(&sub_cmd, &node);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

void setup_debug_message() {
  std_msgs__msg__String__init(&msg_debug);
  const unsigned int CAP = 160;
  msg_debug.data.data = (char*)malloc(CAP);
  msg_debug.data.capacity = CAP;
  msg_debug.data.size = 0;
}

// --------------------------------- Setup/Loop -----------------------------

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(50);
  set_microros_transports(); // nutzt Serial

  if (DIAG_LED >= 0) { pinMode(DIAG_LED, OUTPUT); digitalWrite(DIAG_LED, LOW); }

  for (int i = 0; i < AXIS_COUNT; ++i) {
    STEPS_PER_RAD[i] = (STEPS_PER_REV[i] * MICROSTEPPING[i] * GEAR_RATIO[i]) / (2.0f * M_PI);
  }

  // Precompute default ticks per step from default speeds
  for (int i = 0; i < AXIS_COUNT; ++i) {
    float sps = DEFAULT_SPEED_RAD_S[i] * STEPS_PER_RAD[i];
    DEFAULT_TICKS_PER_STEP[i] = speed_to_ticks(sps);
  }

  for (int i = 0; i < AXIS_COUNT; ++i) {
    pinMode(STEP_PIN[i], OUTPUT);
    pinMode(DIR_PIN[i],  OUTPUT);
    if (ENA_PIN[i] >= 0) pinMode(ENA_PIN[i], OUTPUT);
    digitalWrite(STEP_PIN[i], LOW);
    digitalWrite(DIR_PIN[i],  LOW);
  }
  set_motors_enabled(true);

  memset(axis_rt,  0, sizeof(axis_rt));
  memset(axis_cmd, 0, sizeof(axis_cmd));
  memset(axis_pub, 0, sizeof(axis_pub));
  memset((void*)isr_step_count, 0, sizeof(isr_step_count));
  memset((void*)isr_dir_change_count, 0, sizeof(isr_dir_change_count));
  memset((void*)isr_cmd_applied_count, 0, sizeof(isr_cmd_applied_count));
  memset(&msg_cmd,       0, sizeof(msg_cmd));
  memset(&msg_state_pub, 0, sizeof(msg_state_pub));

  setup_subscriber_message();
  setup_publisher_message();
  setup_debug_message();

  stepTimer = timerBegin(0, 80, true);                 // 80 MHz / 80 = 1 MHz
  timerAttachInterrupt(stepTimer, &onStepTimer, true); // edge
  timerAlarmWrite(stepTimer, TIMER_TICK_US, true);     // ISR alle 50 µs
  timerAlarmEnable(stepTimer);

  // Boot-Info (wird nach erfolgreicher micro-ROS Verbindung per Topic gepublished)
  // Hinweis: Kein Serial.print() hier, da Serial als micro-ROS Transport genutzt wird.

#if BOOT_TEST_MOVE
  delay(1000);
  axis_cmd[0].target_pos = 2000;
  axis_cmd[0].new_cmd = true;
#endif

  // micro-ROS Task auf Core 1 starten
  xTaskCreatePinnedToCore(
      microRosTask,         /* Task function. */
      "microRosTask",       /* name of task. */
      10000,                /* Stack size of task */
      NULL,                 /* parameter of the task */
      1,                    /* priority of the task */
      &microRosTaskHandle,  /* Task handle to keep track of created task */
      1);                   /* pin task to core 1 */
}

void loop() {
  // Der Haupt-Loop auf Core 0 tut fast nichts. Der Watchdog wird vom Idle-Task gefüttert.
  // Die ISR läuft weiter auf Core 0.
  delay(100);
}

// ============================================================================
//  micro-ROS Task (läuft auf Core 1)
// ============================================================================
void microRosTask(void* pvParameters) {
  (void)pvParameters;

  bool connected = false;
  uint32_t last_check_ms = 0;
  int ping_fail_count = 0;

  for (;;) {
    if (!connected) {
      // Transport sicherstellen (Serial)
      set_microros_transports();
      // Agent anpingen (mehrfach), danach Entities erstellen
      if (rmw_uros_ping_agent(200, 5) == RMW_RET_OK) {
        create_entities();
        connected = true;
        ping_fail_count = 0;
        last_check_ms = millis();

        // Boot-Info nach erfolgreicher Verbindung
        char boot[96];
        snprintf(boot, sizeof(boot),
                 "BOOT: reason=%lu QoS=BestEffort Tick=%dus PulseTicks=%d",
                 (unsigned long)esp_reset_reason(), (int)TIMER_TICK_US, (int)((PULSE_TICKS>0)?PULSE_TICKS:1));
        debug_publish(boot);
      } else {
        delay(500);
        continue;
      }
    }

    // Verbunden: Executor ausführen (kurz, nicht blockierend)
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
    delay(5);

    // Verbindung selten überwachen (alle 2 s) und nur bei anhaltendem Ausfall trennen
    uint32_t now = millis();
    if ((now - last_check_ms) >= 2000) {
      last_check_ms = now;
      if (rmw_uros_ping_agent(1000, 2) != RMW_RET_OK) {
        ping_fail_count++;
        if (ping_fail_count >= 5) {
          // Verbindung als verloren betrachten: Entities sauber abbauen und neu versuchen
          destroy_entities();
          connected = false;
          ping_fail_count = 0;
          delay(200);
        }
      } else {
        ping_fail_count = 0;
      }
    }
  }
}
