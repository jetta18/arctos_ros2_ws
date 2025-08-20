// ============================================================================
//  Minimal Arctos ESP32 Firmware - Single Core, Basic ISR + micro-ROS
//  Stripped down version to eliminate boot crashes
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

// ---- Reset-Reason API ----
#include <esp_system.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// ----------------------------- Konfiguration -----------------------------

#define AXIS_COUNT 6
#define SERIAL_BAUD 115200

// Diagnose-LED
#define DIAG_LED 2

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

// Bewegungslimits
const float MAX_SPEED_RAD_S     [AXIS_COUNT] = { 2.0f, 1.0f, 1.0f, 2.5f, 2.5f, 4.0f };
const float DEFAULT_SPEED_RAD_S [AXIS_COUNT] = { 0.8f, 0.5f, 0.5f, 1.0f, 1.0f, 2.0f };

// Step-Timing
static const int  MIN_PULSE_WIDTH_US = 10;
static const int  TIMER_TICK_US      = 50; // ISR alle 50 µs
static const int  PULSE_TICKS        = (MIN_PULSE_WIDTH_US + TIMER_TICK_US - 1) / TIMER_TICK_US;
static const int  MIN_TICKS_PER_STEP = 1;

// ----------------------- Abgeleitete Größen / Zustände --------------------

float STEPS_PER_RAD[AXIS_COUNT];

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
  volatile float steps_per_s;
  volatile bool  new_cmd;
} axis_cmd_t;

axis_cmd_t axis_cmd[AXIS_COUNT];

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

// IRAM-sichere GPIO-Helper
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

uint32_t speed_to_ticks(float steps_per_s) {
  if (steps_per_s <= 0.0f) return 0x7FFFFFFF;
  const float timer_hz = 1e6f / (float)TIMER_TICK_US;
  float ticks = timer_hz / steps_per_s;
  if (ticks < (float)MIN_TICKS_PER_STEP) ticks = (float)MIN_TICKS_PER_STEP;
  if (ticks > (float)0x7FFFFFFF) ticks = (float)0x7FFFFFFF;
  return (uint32_t)lroundf(ticks);
}

// Minimal ISR
void IRAM_ATTR onStepTimer() {
  for (int i = 0; i < AXIS_COUNT; ++i) {
    // 1) Apply new commands
    if (axis_cmd[i].new_cmd) {
      axis_rt[i].target_pos = axis_cmd[i].target_pos;
      axis_rt[i].ticks_per_step = speed_to_ticks(axis_cmd[i].steps_per_s);
      axis_rt[i].moving = (axis_rt[i].target_pos != axis_rt[i].current_pos);
      if (axis_rt[i].tick_counter <= 0) axis_rt[i].tick_counter = axis_rt[i].ticks_per_step;
      axis_cmd[i].new_cmd = false;
    }

    // 2) End STEP pulse
    if (axis_rt[i].pulse_ticks) {
      axis_rt[i].pulse_ticks--;
      if (axis_rt[i].pulse_ticks == 0) {
        gpio_clr_fast(STEP_PIN[i]);
      }
    }

    // 3) Generate steps
    if (axis_rt[i].moving) {
      axis_rt[i].tick_counter--;
      if (axis_rt[i].tick_counter <= 0) {
        bool dir_needed = (axis_rt[i].target_pos > axis_rt[i].current_pos);
        
        // Set direction if changed
        if (dir_needed != axis_rt[i].dir_state) {
          axis_rt[i].dir_state = dir_needed;
          bool out = dir_needed ^ DIR_INVERT[i];
          gpio_dir_fast(DIR_PIN[i], out);
          axis_rt[i].tick_counter += axis_rt[i].ticks_per_step; // Wait one period
          continue;
        }

        // Generate step
        gpio_set_fast(STEP_PIN[i]);
        axis_rt[i].pulse_ticks = (PULSE_TICKS > 0) ? PULSE_TICKS : 1;

        // Update position
        axis_rt[i].current_pos += dir_needed ? +1 : -1;
        axis_rt[i].tick_counter += axis_rt[i].ticks_per_step;

        // Check if target reached
        if (axis_rt[i].current_pos == axis_rt[i].target_pos) {
          axis_rt[i].moving = false;
        }
      }
    }
  }
}

// ------------------------------ Utilities ---------------------------------

void set_motors_enabled(bool enable) {
  for (int i = 0; i < AXIS_COUNT; ++i) {
    if (ENA_PIN[i] >= 0) {
      digitalWrite(ENA_PIN[i], enable ? LOW : HIGH);
    }
  }
}

long   rad_to_steps (int a, double rad)      { return (a<0||a>=AXIS_COUNT)?0:(long) llround(rad * STEPS_PER_RAD[a]); }
double steps_to_rad (int a, long steps)      { return (a<0||a>=AXIS_COUNT)?0.0: (double)steps / STEPS_PER_RAD[a]; }
float  radps_to_sps (int a, double radps)    { return (a<0||a>=AXIS_COUNT)?0.0f:(float)(fabs(radps)*STEPS_PER_RAD[a]); }

// ------------------------------ ROS Callbacks -----------------------------

static uint32_t cmd_count = 0;

void debug_publish(const char* s) {
  size_t n = strlen(s);
  if (n >= msg_debug.data.capacity) n = msg_debug.data.capacity - 1;
  memcpy(msg_debug.data.data, s, n);
  msg_debug.data.data[n] = 0;
  msg_debug.data.size = n;
  rcl_publish(&pub_debug, &msg_debug, NULL);
}

void cmd_callback(const void* msgin) {
  if (DIAG_LED >= 0) { 
    digitalWrite(DIAG_LED, HIGH); 
    delayMicroseconds(10); 
    digitalWrite(DIAG_LED, LOW); 
  }

  const auto* m = (const sensor_msgs__msg__JointState*)msgin;
  const size_t n_pos = m->position.size;
  const size_t n_vel = m->velocity.size;
  const size_t n_name = m->name.size;
  if (n_pos == 0) return;

  ++cmd_count;

  auto apply = [&](int axis, size_t idx){
    double pos = (idx < n_pos) ? m->position.data[idx] : 0.0;
    double vel = (idx < n_vel) ? m->velocity.data[idx] : 0.0;

    long  tgt_steps = rad_to_steps(axis, pos);
    float sps = radps_to_sps(axis, vel);
    float mech_max = MAX_SPEED_RAD_S[axis] * STEPS_PER_RAD[axis];
    if (sps <= 0.0f) sps = DEFAULT_SPEED_RAD_S[axis] * STEPS_PER_RAD[axis];
    if (sps > mech_max) sps = mech_max;

    axis_cmd[axis].target_pos = tgt_steps;
    axis_cmd[axis].steps_per_s = sps;
    axis_cmd[axis].new_cmd = true;
  };

  if (n_name == AXIS_COUNT) {
    for (size_t i = 0; i < n_name; ++i) {
      const char* nm = m->name.data[i].data;
      for (int a = 0; a < AXIS_COUNT; ++a) {
        if (strcmp(nm, JOINT_NAMES[a]) == 0) { apply(a, i); break; }
      }
    }
  } else {
    size_t n = (n_pos < (size_t)AXIS_COUNT) ? n_pos : (size_t)AXIS_COUNT;
    for (size_t i = 0; i < n; ++i) apply((int)i, i);
  }

  char buf[120];
  snprintf(buf, sizeof(buf), "[cmd #%lu] ax0: tgt=%ld sps=%.0f",
           (unsigned long)cmd_count, (long)axis_cmd[0].target_pos, (double)axis_cmd[0].steps_per_s);
  debug_publish(buf);
}

void pub_timer_callback(rcl_timer_t*, int64_t) {
  uint64_t now_ns = rmw_uros_epoch_nanos();
  if (now_ns > 0) {
    msg_state_pub.header.stamp.sec = (int32_t)(now_ns / 1000000000ULL);
    msg_state_pub.header.stamp.nanosec = (uint32_t)(now_ns % 1000000000ULL);
  } else {
    uint32_t ms = millis();
    msg_state_pub.header.stamp.sec = (int32_t)(ms / 1000);
    msg_state_pub.header.stamp.nanosec = (uint32_t)((ms % 1000) * 1000000UL);
  }

  for (int i = 0; i < AXIS_COUNT; ++i) {
    pub_positions[i] = steps_to_rad(i, axis_rt[i].current_pos);
    pub_velocities[i] = axis_rt[i].moving ? 
      (1e6f / (float)(axis_rt[i].ticks_per_step * TIMER_TICK_US)) / STEPS_PER_RAD[i] : 0.0f;
  }
  rcl_publish(&pub_state, &msg_state_pub, NULL);

  char hb[100];
  snprintf(hb, sizeof(hb), "HB X:pos=%ld mov=%d | Y:pos=%ld mov=%d",
           (long)axis_rt[0].current_pos, (int)axis_rt[0].moving,
           (long)axis_rt[1].current_pos, (int)axis_rt[1].moving);
  debug_publish(hb);
}

// --------------------------- ROS Entities/Buffer --------------------------

void setup_publisher_message() {
  msg_state_pub.name.data = pub_names;
  msg_state_pub.name.capacity = AXIS_COUNT;
  msg_state_pub.name.size = AXIS_COUNT;
  for (int i = 0; i < AXIS_COUNT; ++i) {
    pub_names[i].data = (char*)JOINT_NAMES[i];
    pub_names[i].capacity = strlen(JOINT_NAMES[i]) + 1;
    pub_names[i].size = strlen(JOINT_NAMES[i]);
  }
  msg_state_pub.position.data = pub_positions;
  msg_state_pub.position.capacity = AXIS_COUNT;
  msg_state_pub.position.size = AXIS_COUNT;

  msg_state_pub.velocity.data = pub_velocities;
  msg_state_pub.velocity.capacity = AXIS_COUNT;
  msg_state_pub.velocity.size = AXIS_COUNT;

  msg_state_pub.effort.data = NULL;
  msg_state_pub.effort.capacity = 0;
  msg_state_pub.effort.size = 0;

  msg_state_pub.header.frame_id.data = NULL;
  msg_state_pub.header.frame_id.size = 0;
  msg_state_pub.header.frame_id.capacity = 0;
}

void setup_subscriber_message() {
  msg_cmd.position.data = sub_positions;
  msg_cmd.position.capacity = AXIS_COUNT;
  msg_cmd.position.size = 0;

  msg_cmd.velocity.data = sub_velocities;
  msg_cmd.velocity.capacity = AXIS_COUNT;
  msg_cmd.velocity.size = 0;

  msg_cmd.name.data = sub_names;
  msg_cmd.name.capacity = AXIS_COUNT;
  msg_cmd.name.size = 0;
  for (int i = 0; i < AXIS_COUNT; ++i) {
    sub_names[i].data = sub_name_storage[i];
    sub_names[i].capacity = sizeof(sub_name_storage[i]);
    sub_names[i].size = 0;
  }

  msg_cmd.effort.data = NULL;
  msg_cmd.effort.capacity = 0;
  msg_cmd.effort.size = 0;

  msg_cmd.header.frame_id.data = sub_frame_id_buf;
  msg_cmd.header.frame_id.capacity = sizeof(sub_frame_id_buf);
  msg_cmd.header.frame_id.size = 0;
}

void setup_debug_message() {
  std_msgs__msg__String__init(&msg_debug);
  const unsigned int CAP = 160;
  msg_debug.data.data = (char*)malloc(CAP);
  msg_debug.data.capacity = CAP;
  msg_debug.data.size = 0;
}

void create_entities() {
  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "arctos_esp32_firmware", "", &support);

  rclc_subscription_init_best_effort(&sub_cmd, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState), JOINT_COMMANDS_TOPIC);
  rclc_publisher_init_best_effort(&pub_state, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState), JOINT_STATES_TOPIC);
  rclc_publisher_init_best_effort(&pub_debug, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), DEBUG_TOPIC);

  const unsigned int pub_period_ms = 100;
  rclc_timer_init_default(&pub_timer, &support, RCL_MS_TO_NS(pub_period_ms), pub_timer_callback);

  rclc_executor_init(&executor, &support.context, 3, &allocator);
  rclc_executor_add_subscription(&executor, &sub_cmd, &msg_cmd, &cmd_callback, ON_NEW_DATA);
  rclc_executor_add_timer(&executor, &pub_timer);
}

// --------------------------------- Setup/Loop -----------------------------

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(100);
  set_microros_transports();

  if (DIAG_LED >= 0) { pinMode(DIAG_LED, OUTPUT); digitalWrite(DIAG_LED, LOW); }

  for (int i = 0; i < AXIS_COUNT; ++i) {
    STEPS_PER_RAD[i] = (STEPS_PER_REV[i] * MICROSTEPPING[i] * GEAR_RATIO[i]) / (2.0f * M_PI);
  }

  for (int i = 0; i < AXIS_COUNT; ++i) {
    pinMode(STEP_PIN[i], OUTPUT);
    pinMode(DIR_PIN[i], OUTPUT);
    if (ENA_PIN[i] >= 0) pinMode(ENA_PIN[i], OUTPUT);
    digitalWrite(STEP_PIN[i], LOW);
    digitalWrite(DIR_PIN[i], LOW);
  }
  set_motors_enabled(true);

  memset(axis_rt, 0, sizeof(axis_rt));
  memset(axis_cmd, 0, sizeof(axis_cmd));
  memset(&msg_cmd, 0, sizeof(msg_cmd));
  memset(&msg_state_pub, 0, sizeof(msg_state_pub));

  setup_subscriber_message();
  setup_publisher_message();
  setup_debug_message();

  stepTimer = timerBegin(0, 80, true);
  timerAttachInterrupt(stepTimer, &onStepTimer, true);
  timerAlarmWrite(stepTimer, TIMER_TICK_US, true);
  timerAlarmEnable(stepTimer);

  // Initialize micro-ROS
  create_entities();
  
  // Boot message
  char boot[80];
  snprintf(boot, sizeof(boot), "BOOT: Minimal firmware ready, tick=%dus", TIMER_TICK_US);
  debug_publish(boot);
}

void loop() {
  // Simple single-core approach - just spin the executor
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
  delay(10);
}
