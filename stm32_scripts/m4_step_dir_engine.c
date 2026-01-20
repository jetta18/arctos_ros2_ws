/**
 ******************************************************************************
 * @file           : m4_step_dir_engine.c
 * @brief          : M4 Core - Step/Dir Pulse Generation Engine
 * @author         : STM32 H755 Dual-Core Implementation
 ******************************************************************************
 * This file implements a high-performance step/dir pulse generation engine
 * for 6-axis stepper motor control on the Cortex-M4 core.
 * 
 * The M7 core handles ROS2 communication and converts joint angles to steps.
 * The M4 core generates precise step/dir pulses using hardware timers.
 ******************************************************************************
 */

#include "main.h"
#include <stdbool.h>
#include <math.h>

/* Motion segment buffer config (must match M7) */
#define SEGMENT_BUFFER_LEN 128U

typedef struct {
  uint32_t duration_us;
  int32_t step_inc[6];
} MotionSegment;

/* Shared memory structure - must match M7 definition */
typedef struct {
  int32_t target_steps[6];
  int32_t current_steps[6];
  uint32_t max_speed[6];
  uint32_t acceleration[6];
  uint8_t motor_enable;
  volatile uint8_t emergency_stop;
  volatile uint8_t data_valid;
  volatile uint8_t m4_ready;
  float gear_ratios[6];
  uint32_t steps_per_rev[6];
  uint8_t direction_inverted;
  
  volatile uint16_t seg_head;
  volatile uint16_t seg_tail;
  uint16_t seg_capacity;
  MotionSegment seg_buf[SEGMENT_BUFFER_LEN];
} __attribute__((aligned(32))) SharedMotorData_t;

/* Access shared memory from M7 (placed in D3 SRAM at 0x38000000) */
#define SHARED_MEMORY_BASE 0x38000000
#define shared_motor_data (*((SharedMotorData_t*)SHARED_MEMORY_BASE))

/* Motor state for each axis */
typedef struct {
  int32_t current_position;      // Current position in steps
  int32_t target_position;       // Target position in steps
  int32_t steps_to_move;         // Remaining steps to move
  uint32_t current_speed;        // Current speed in steps/s
  uint32_t step_delay_us;        // Delay between steps in microseconds
  int8_t direction;              // 1 = forward, -1 = backward
  bool moving;                   // Motor is currently moving
  uint32_t accel_steps;          // Steps for acceleration phase
  uint32_t decel_steps;          // Steps for deceleration phase
} MotorState_t;

MotorState_t motor_states[6] = {0};

/* GPIO Pin definitions for Step/Dir signals */
/* TODO: Configure these based on your actual hardware connections */
typedef struct {
  GPIO_TypeDef* step_port;
  uint16_t step_pin;
  GPIO_TypeDef* dir_port;
  uint16_t dir_pin;
} MotorPins_t;

MotorPins_t motor_pins[6] = {
  // Motor 0 (X_joint)
  {GPIOA, GPIO_PIN_0, GPIOA, GPIO_PIN_1},
  // Motor 1 (Y_joint)
  {GPIOA, GPIO_PIN_2, GPIOA, GPIO_PIN_3},
  // Motor 2 (Z_joint)
  {GPIOA, GPIO_PIN_4, GPIOA, GPIO_PIN_5},
  // Motor 3 (A_joint)
  {GPIOA, GPIO_PIN_6, GPIOA, GPIO_PIN_7},
  // Motor 4 (B_joint)
  {GPIOB, GPIO_PIN_0, GPIOB, GPIO_PIN_1},
  // Motor 5 (C_joint)
  {GPIOB, GPIO_PIN_2, GPIOB, GPIO_PIN_3},
};

/* Hardware timer for step pulse generation */
TIM_HandleTypeDef htim6;  // Use TIM6 for step/event timing (variable-period)

/* Scheduler state for current segment */
static volatile uint32_t seg_elapsed_us = 0;
static volatile bool seg_active = false;
static MotionSegment cur_seg;
static uint32_t next_us[6] = {0};
static uint32_t inter_us[6] = {0};
static uint32_t rem_steps[6] = {0};
static int8_t step_dir_sign[6] = {0};

/* Forward declarations */
static bool fetch_next_segment(MotionSegment* out);
static void load_segment(const MotionSegment* s);
static void schedule_next_event(void);
static inline void generate_step_pulse_mask(uint8_t mask);

/**
 * @brief Initialize M4 core and step/dir engine
 */
void M4_StepDir_Init(void)
{
  /* Initialize GPIO pins for Step/Dir outputs */
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  
  for (int i = 0; i < 6; i++) {
    // Configure STEP pins
    GPIO_InitStruct.Pin = motor_pins[i].step_pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(motor_pins[i].step_port, &GPIO_InitStruct);
    
    // Configure DIR pins
    GPIO_InitStruct.Pin = motor_pins[i].dir_pin;
    HAL_GPIO_Init(motor_pins[i].dir_port, &GPIO_InitStruct);
    
    // Initialize to LOW
    HAL_GPIO_WritePin(motor_pins[i].step_port, motor_pins[i].step_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(motor_pins[i].dir_port, motor_pins[i].dir_pin, GPIO_PIN_RESET);
  }
  
  /* Initialize timer for step/event scheduling @ ~1MHz timebase */
  __HAL_RCC_TIM6_CLK_ENABLE();
  
  htim6.Instance = TIM6;
  uint32_t pclk = HAL_RCC_GetPCLK1Freq();
  uint32_t psc = (pclk / 1000000U);
  if (psc == 0U) psc = 1U;
  htim6.Init.Prescaler = (uint32_t)(psc - 1U);  // ~1 us per tick
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 1000 - 1;                 // initial 1ms, will be reprogrammed
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK) {
    Error_Handler();
  }
  
  /* Enable timer interrupt */
  HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
  
  /* Initialize runtime state */
  seg_elapsed_us = 0;
  seg_active = false;
  for (int i = 0; i < 6; i++) {
    motor_states[i].current_position = 0;
  }
  
  /* Signal M7 that M4 is ready */
  shared_motor_data.m4_ready = 1;

  /* Prime first event and start timer */
  schedule_next_event();
  HAL_TIM_Base_Start_IT(&htim6);
}

/* Fetch next segment from shared ring buffer (returns true if one was fetched) */
static bool fetch_next_segment(MotionSegment* out)
{
  bool ok = false;
  if (HAL_HSEM_FastTake(HSEM_ID_0) == HAL_OK) {
    uint16_t head = shared_motor_data.seg_head;
    uint16_t tail = shared_motor_data.seg_tail;
    uint16_t cap  = shared_motor_data.seg_capacity ? shared_motor_data.seg_capacity : (uint16_t)SEGMENT_BUFFER_LEN;
    if (head != tail) {
      *out = shared_motor_data.seg_buf[tail];
      tail = (uint16_t)((tail + 1U) % cap);
      shared_motor_data.seg_tail = tail;
      ok = true;
    }
    HAL_HSEM_Release(HSEM_ID_0, 0);
  }
  return ok;
}

/* Load a segment into runtime scheduler */
static void load_segment(const MotionSegment* s)
{
  cur_seg = *s;
  seg_elapsed_us = 0;
  seg_active = true;
  for (int i = 0; i < 6; i++) {
    int32_t inc = cur_seg.step_inc[i];
    if (inc == 0) {
      rem_steps[i] = 0;
      inter_us[i] = 0xFFFFFFFFu;
      next_us[i] = 0xFFFFFFFFu;
      step_dir_sign[i] = 0;
      continue;
    }
    /* Direction */
    bool dir_positive = (inc > 0);
    if (shared_motor_data.direction_inverted & (1U << i)) dir_positive = !dir_positive;
    HAL_GPIO_WritePin(motor_pins[i].dir_port,
                      motor_pins[i].dir_pin,
                      dir_positive ? GPIO_PIN_SET : GPIO_PIN_RESET);

    uint32_t steps = (uint32_t)(inc > 0 ? inc : -inc);
    rem_steps[i] = steps;
    /* Evenly distribute across duration */
    inter_us[i] = (steps > 0U) ? (cur_seg.duration_us / steps) : 0xFFFFFFFFu;
    if (inter_us[i] == 0U) inter_us[i] = 1U; // clamp to 1us
    next_us[i] = inter_us[i] / 2U;           // center pulses
    step_dir_sign[i] = dir_positive ? 1 : -1;
  }
}

/* Generate pulses for all axes indicated by mask with ~1us pulse width */
static inline void generate_step_pulse_mask(uint8_t mask)
{
  // Rising edges
  for (int i = 0; i < 6; i++) {
    if (mask & (1U << i)) {
      HAL_GPIO_WritePin(motor_pins[i].step_port, motor_pins[i].step_pin, GPIO_PIN_SET);
    }
  }
  // Minimal delay (~1us)
  for (volatile int d = 0; d < 240; d++) { __NOP(); }
  // Falling edges
  for (int i = 0; i < 6; i++) {
    if (mask & (1U << i)) {
      HAL_GPIO_WritePin(motor_pins[i].step_port, motor_pins[i].step_pin, GPIO_PIN_RESET);
    }
  }
}

/* Decide and program next timer event */
static void schedule_next_event(void)
{
  /* Emergency stop: nothing scheduled */
  if (shared_motor_data.emergency_stop) {
    __HAL_TIM_DISABLE(&htim6);
    return;
  }

  /* If no active segment, try to fetch one */
  if (!seg_active) {
    MotionSegment s;
    if (fetch_next_segment(&s)) {
      load_segment(&s);
    }
  }

  uint32_t next_delta = 1000U; // default idle 1ms
  bool have_event = false;

  if (seg_active) {
    // Find nearest next pulse across axes
    uint32_t min_due = 0xFFFFFFFFu;
    for (int i = 0; i < 6; i++) {
      if (rem_steps[i] > 0U && next_us[i] >= seg_elapsed_us) {
        if (next_us[i] < min_due) min_due = next_us[i];
      }
    }
    if (min_due != 0xFFFFFFFFu) {
      have_event = true;
      uint32_t elapsed = seg_elapsed_us;
      next_delta = (min_due > elapsed) ? (min_due - elapsed) : 1U;
    } else {
      // No more steps; wait until segment end or fetch next immediately
      if (seg_elapsed_us < cur_seg.duration_us) {
        have_event = true;
        next_delta = cur_seg.duration_us - seg_elapsed_us;
      } else {
        // Segment finished
        seg_active = false;
        // Try to immediately load next segment
        MotionSegment s;
        if (fetch_next_segment(&s)) {
          load_segment(&s);
          // Recurse to compute next_delta for new segment
          schedule_next_event();
          return;
        }
      }
    }
  }

  // Program timer for next event
  if (!have_event) {
    next_delta = 1000U; // idle 1ms
  }
  if (next_delta == 0U) next_delta = 1U;
  __HAL_TIM_DISABLE(&htim6);
  __HAL_TIM_SET_AUTORELOAD(&htim6, (uint32_t)(next_delta - 1U));
  __HAL_TIM_SET_COUNTER(&htim6, 0U);
  __HAL_TIM_ENABLE(&htim6);
}

/**
 * @brief Generate step pulse for a motor
 */
static inline void generate_step_pulse(int motor_idx)
{
  // Generate step pulse (minimum 1us pulse width)
  HAL_GPIO_WritePin(motor_pins[motor_idx].step_port, 
                    motor_pins[motor_idx].step_pin, 
                    GPIO_PIN_SET);
  
  // Small delay (adjust based on driver requirements)
  for (volatile int i = 0; i < 10; i++);
  
  HAL_GPIO_WritePin(motor_pins[motor_idx].step_port, 
                    motor_pins[motor_idx].step_pin, 
                    GPIO_PIN_RESET);
}

/**
 * @brief Timer interrupt handler - generates step pulses
 */
void TIM6_DAC_IRQHandler(void)
{
  if (__HAL_TIM_GET_FLAG(&htim6, TIM_FLAG_UPDATE) != RESET) {
    if (__HAL_TIM_GET_IT_SOURCE(&htim6, TIM_IT_UPDATE) != RESET) {
      __HAL_TIM_CLEAR_IT(&htim6, TIM_IT_UPDATE);

      // Advance segment elapsed time by programmed interval
      uint32_t delta = __HAL_TIM_GET_AUTORELOAD(&htim6) + 1U;
      seg_elapsed_us += delta;

      // Determine which axes need a step at this time
      uint8_t pulse_mask = 0;
      for (int i = 0; i < 6; i++) {
        if (rem_steps[i] > 0U && next_us[i] <= seg_elapsed_us) {
          pulse_mask |= (1U << i);
        }
      }

      if (pulse_mask) {
        generate_step_pulse_mask(pulse_mask);
        // Update per-axis state
        for (int i = 0; i < 6; i++) {
          if (pulse_mask & (1U << i)) {
            // Position bookkeeping for feedback (segment direction)
            motor_states[i].current_position += step_dir_sign[i] > 0 ? 1 : -1;
            if (rem_steps[i] > 0U) rem_steps[i]--;
            next_us[i] += inter_us[i];
          }
        }
      }

      // If segment done, try to fetch and load next one
      bool any_remaining = false;
      for (int i = 0; i < 6; i++) { if (rem_steps[i] > 0U) { any_remaining = true; break; } }
      if (!any_remaining && seg_active && (seg_elapsed_us >= cur_seg.duration_us)) {
        seg_active = false;
      }

      // Periodically write current positions back (best-effort, no lock here)
      if (HAL_HSEM_FastTake(HSEM_ID_0) == HAL_OK) {
        for (int i = 0; i < 6; i++) {
          shared_motor_data.current_steps[i] = motor_states[i].current_position;
        }
        HAL_HSEM_Release(HSEM_ID_0, 0);
      }

      // Schedule next event
      schedule_next_event();
    }
  }
}

/**
 * @brief Main loop for M4 core
 */
void M4_Main_Loop(void)
{
  while (1) {
    // Idle loop: all scheduling happens in TIM6 ISR
    HAL_Delay(1);
  }
}
