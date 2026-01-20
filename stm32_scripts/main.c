/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <math.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>

#include <std_msgs/msg/int32.h>
#include <sensor_msgs/msg/joint_state.h>
/* USER CODE END Includes */

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* Motion segment configuration */
#define SEGMENT_DURATION_US 5000U
#define SEGMENT_BUFFER_LEN 128U

/* DUAL_CORE_BOOT_SYNC_SEQUENCE: Define for dual core boot synchronization    */
/*                             demonstration code based on hardware semaphore */
/* This define is present in both CM7/CM4 projects                            */
/* To comment when developping/debugging on a single core                     */
#define DUAL_CORE_BOOT_SYNC_SEQUENCE

#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
uint32_t defaultTaskBuffer[ 3000 ];
osStaticThreadDef_t defaultTaskControlBlock;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .cb_mem = &defaultTaskControlBlock,
  .cb_size = sizeof(defaultTaskControlBlock),
  .stack_mem = &defaultTaskBuffer[0],
  .stack_size = sizeof(defaultTaskBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */

// Joint angle storage (6 joints: X, Y, Z, A, B, C)
typedef struct {
  double position[6];
  double velocity[6];
  uint32_t last_update_time;
  bool data_received;
} JointData_t;

JointData_t joint_data = {0};

/* Motion segment type for ring buffer */
typedef struct {
  uint32_t duration_us;
  int32_t step_inc[6];
} MotionSegment;

/* Last committed command steps and time for segmentizer */
static int32_t last_cmd_steps[6] = {0};
static uint32_t last_cmd_time_ms = 0;

// Shared memory structure for M7-M4 communication (Step/Dir generation)
// Place in shared SRAM (D3 domain) for dual-core access
typedef struct {
  // Target positions in motor steps for each joint
  int32_t target_steps[6];
  
  // Current positions in motor steps (feedback from M4)
  int32_t current_steps[6];
  
  // Maximum speed in steps/second for each joint
  uint32_t max_speed[6];
  
  // Acceleration in steps/s^2 for each joint
  uint32_t acceleration[6];
  
  // Motor enable flags (bit per motor)
  uint8_t motor_enable;
  
  // Emergency stop flag
  volatile uint8_t emergency_stop;
  
  // Data valid flag (set by M7, cleared by M4 after processing)
  volatile uint8_t data_valid;
  
  // M4 ready flag
  volatile uint8_t m4_ready;
  
  // Gear ratios (for reference)
  float gear_ratios[6];
  
  // Steps per revolution for each motor
  uint32_t steps_per_rev[6];
  
  // Direction inversion flags
  uint8_t direction_inverted;
  
  // Motion segment ring buffer (M7 producer -> M4 consumer)
  volatile uint16_t seg_head;
  volatile uint16_t seg_tail;
  uint16_t seg_capacity;
  MotionSegment seg_buf[SEGMENT_BUFFER_LEN];
  
} __attribute__((aligned(32))) SharedMotorData_t;

// Place shared data in D3 SRAM (0x38000000) for dual-core access
#if defined ( __ICCARM__ )
#pragma location = 0x38000000
SharedMotorData_t shared_motor_data;
#elif defined ( __GNUC__ )
SharedMotorData_t shared_motor_data __attribute__((section(".shared_data")));
#endif

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART3_UART_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern struct netif gnetif;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
/* USER CODE BEGIN Boot_Mode_Sequence_0 */
#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
  int32_t timeout;
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */
/* USER CODE END Boot_Mode_Sequence_0 */

/* USER CODE BEGIN Boot_Mode_Sequence_1 */
#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
  /* Wait until CPU2 boots and enters in stop mode or timeout*/
  timeout = 0xFFFF;
  while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) != RESET) && (timeout-- > 0));
  if ( timeout < 0 )
  {
  Error_Handler();
  }
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */
/* USER CODE END Boot_Mode_Sequence_1 */
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();
/* USER CODE BEGIN Boot_Mode_Sequence_2 */
#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
/* When system initialization is finished, Cortex-M7 will release Cortex-M4 by means of
HSEM notification */
/*HW semaphore Clock enable*/
__HAL_RCC_HSEM_CLK_ENABLE();
/*Take HSEM */
HAL_HSEM_FastTake(HSEM_ID_0);
/*Release HSEM in order to notify the CPU2(CM4)*/
HAL_HSEM_Release(HSEM_ID_0,0);
/* wait until CPU2 wakes up from stop mode */
timeout = 0xFFFF;
while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) == RESET) && (timeout-- > 0));
if ( timeout < 0 )
{
Error_Handler();
}
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */
/* USER CODE END Boot_Mode_Sequence_2 */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 5;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_EXTERNAL1;
  sSlaveConfig.InputTrigger = TIM_TS_ITR1;
  if (HAL_TIM_SlaveConfigSynchro(&htim2, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pins : PC1 PC4 PC5 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA2 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG1_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PG11 PG13 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
bool cubemx_transport_open(struct uxrCustomTransport * transport);
bool cubemx_transport_close(struct uxrCustomTransport * transport);
size_t cubemx_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);

void * microros_allocate(size_t size, void * state);
void microros_deallocate(void * pointer, void * state);
void * microros_reallocate(void * pointer, size_t size, void * state);
void * microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state);

// Callback function for joint commands subscriber
void joint_commands_callback(const void * msgin)
{
  const sensor_msgs__msg__JointState * msg = (const sensor_msgs__msg__JointState *)msgin;
  
  // Update timestamp
  joint_data.last_update_time = HAL_GetTick();
  
  // Copy joint positions (up to 6 joints: X, Y, Z, A, B, C)
  size_t num_joints = msg->name.size < 6 ? msg->name.size : 6;
  for (size_t i = 0; i < num_joints; i++) {
    // Copy position data
    if (i < msg->position.size) {
      joint_data.position[i] = msg->position.data[i];
    }
    // Copy velocity data
    if (i < msg->velocity.size) {
      joint_data.velocity[i] = msg->velocity.data[i];
    }
  }
  
  joint_data.data_received = true;
}

// Initialize shared motor data with configuration from ros2_control.xacro
void init_shared_motor_data(void)
{
  // Gear ratios from ros2_control.xacro
  shared_motor_data.gear_ratios[0] = 13.5f;      // X_joint
  shared_motor_data.gear_ratios[1] = 150.0f;    // Y_joint
  shared_motor_data.gear_ratios[2] = 150.0f;    // Z_joint
  shared_motor_data.gear_ratios[3] = 48.0f;     // A_joint
  shared_motor_data.gear_ratios[4] = 27.3375f;  // B_joint
  shared_motor_data.gear_ratios[5] = 10.0f;     // C_joint
  
  // Steps per revolution (200 steps/rev * microstepping)
  // Adjust microstepping factor as needed (1, 2, 4, 8, 16, 32, etc.)
  uint32_t microsteps = 16;  // 16x microstepping
  for (int i = 0; i < 6; i++) {
    shared_motor_data.steps_per_rev[i] = 200 * microsteps;
  }
  
  // Direction inversion flags from ros2_control.xacro
  shared_motor_data.direction_inverted = 0;
  shared_motor_data.direction_inverted |= (0 << 0);  // X_joint: false
  shared_motor_data.direction_inverted |= (1 << 1);  // Y_joint: true
  shared_motor_data.direction_inverted |= (0 << 2);  // Z_joint: false
  shared_motor_data.direction_inverted |= (1 << 3);  // A_joint: true
  shared_motor_data.direction_inverted |= (1 << 4);  // B_joint: true
  shared_motor_data.direction_inverted |= (1 << 5);  // C_joint: true
  
  // Default max speeds (steps/second) derived from 3000 RPM and microstepping
  // max_speed = (RPM / 60) * steps_per_rev
  for (int i = 0; i < 6; i++) {
    shared_motor_data.max_speed[i] = (uint32_t)((3000U * shared_motor_data.steps_per_rev[i]) / 60U); // e.g., 3000rpm * 3200 / 60 = 160000 sps
  }
  
  // Default acceleration (steps/s^2)
  for (int i = 0; i < 6; i++) {
    shared_motor_data.acceleration[i] = 5000;  // 5000 steps/s^2
  }
  
  // Initialize positions
  for (int i = 0; i < 6; i++) {
    shared_motor_data.target_steps[i] = 0;
    shared_motor_data.current_steps[i] = 0;
  }
  
  // Enable all motors by default
  shared_motor_data.motor_enable = 0x3F;  // All 6 motors enabled (bits 0-5)
  
  // Clear flags
  shared_motor_data.emergency_stop = 0;
  shared_motor_data.data_valid = 0;
  shared_motor_data.m4_ready = 0;

  // Init motion segment ring buffer
  shared_motor_data.seg_head = 0;
  shared_motor_data.seg_tail = 0;
  shared_motor_data.seg_capacity = SEGMENT_BUFFER_LEN;
  last_cmd_time_ms = HAL_GetTick();
}

// Convert JointState to motion segments and feed ring buffer for M4
void process_joint_commands(void)
{
  if (!joint_data.data_received) {
    return;
  }
  
  // Timeout safety
  uint32_t now_ms = HAL_GetTick();
  if ((now_ms - joint_data.last_update_time) > 1000U) {
    shared_motor_data.emergency_stop = 1;
    return;
  }

  // Compute desired absolute step targets from joint positions
  int32_t target_steps[6];
  for (int i = 0; i < 6; i++) {
    double pos_rad = joint_data.position[i];
    double motor_revs = (pos_rad / (2.0 * M_PI)) * (double)shared_motor_data.gear_ratios[i];
    double raw_steps = motor_revs * (double)shared_motor_data.steps_per_rev[i];
    int32_t steps = (int32_t)((raw_steps >= 0.0) ? (raw_steps + 0.5) : (raw_steps - 0.5));
    if (shared_motor_data.direction_inverted & (1U << i)) {
      steps = -steps;
    }
    target_steps[i] = steps;
  }

  int32_t remain[6];
  for (int i = 0; i < 6; i++) {
    remain[i] = target_steps[i] - last_cmd_steps[i];
  }

  // Decide how many 5ms segments to produce based on elapsed time
  if (last_cmd_time_ms == 0U) {
    last_cmd_time_ms = now_ms;
  }
  uint32_t dt_ms = now_ms - last_cmd_time_ms;
  uint32_t seg_ms = SEGMENT_DURATION_US / 1000U;
  if (dt_ms < seg_ms) dt_ms = seg_ms;
  uint32_t wanted = dt_ms / seg_ms;
  if (wanted == 0U) wanted = 1U;
  if (wanted > 8U) wanted = 8U; // cap to avoid burst writes

  // Wait until M4 reports ready once
  uint32_t t0 = now_ms;
  while (!shared_motor_data.m4_ready && ((HAL_GetTick() - t0) < 10U)) { }
  if (!shared_motor_data.m4_ready) {
    return;
  }

  // Produce segments into ring buffer
  if (HAL_HSEM_FastTake(HSEM_ID_0) != HAL_OK) {
    return;
  }
  uint16_t head = shared_motor_data.seg_head;
  uint16_t tail = shared_motor_data.seg_tail;
  uint16_t cap  = shared_motor_data.seg_capacity ? shared_motor_data.seg_capacity : (uint16_t)SEGMENT_BUFFER_LEN;
  uint16_t fill = (uint16_t)((head + cap - tail) % cap);
  uint16_t free_slots = (uint16_t)(cap - fill - 1U);
  if (free_slots == 0U) {
    HAL_HSEM_Release(HSEM_ID_0, 0);
    return;
  }
  uint32_t to_produce = wanted;
  if (to_produce > free_slots) to_produce = free_slots;

  for (uint32_t s = 0; s < to_produce; s++) {
    int32_t chunk[6];
    for (int i = 0; i < 6; i++) {
      int32_t segments_left = (int32_t)(to_produce - s);
      int32_t c = (segments_left != 0) ? (remain[i] / segments_left) : remain[i];
      uint32_t max_inc = (uint32_t)(((uint64_t)shared_motor_data.max_speed[i] * (uint64_t)SEGMENT_DURATION_US) / 1000000ULL);
      uint32_t mag = (uint32_t)(c < 0 ? -c : c);
      if (mag > max_inc) {
        c = (c < 0) ? -(int32_t)max_inc : (int32_t)max_inc;
      }
      chunk[i] = c;
      remain[i] -= c;
      last_cmd_steps[i] += c;
    }
    shared_motor_data.seg_buf[head].duration_us = SEGMENT_DURATION_US;
    for (int i = 0; i < 6; i++) {
      shared_motor_data.seg_buf[head].step_inc[i] = chunk[i];
    }
    head = (uint16_t)((head + 1U) % cap);
  }
  shared_motor_data.seg_head = head;
  HAL_HSEM_Release(HSEM_ID_0, 0);
  last_cmd_time_ms = now_ms;
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */

  // micro-ROS configuration

  rmw_uros_set_custom_transport(
    true,
    (void *) &huart3,
    cubemx_transport_open,
    cubemx_transport_close,
    cubemx_transport_write,
    cubemx_transport_read);

  rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
  freeRTOS_allocator.allocate = microros_allocate;
  freeRTOS_allocator.deallocate = microros_deallocate;
  freeRTOS_allocator.reallocate = microros_reallocate;
  freeRTOS_allocator.zero_allocate =  microros_zero_allocate;

  if (!rcutils_set_default_allocator(&freeRTOS_allocator)) {
      printf("Error on default allocators (line %d)\n", __LINE__);
  }

  // micro-ROS app

  rcl_publisher_t publisher;
  rcl_subscription_t subscriber;
  std_msgs__msg__Int32 pub_msg;
  sensor_msgs__msg__JointState sub_msg;
  rclc_support_t support;
  rcl_allocator_t allocator;
  rcl_node_t node;
  rclc_executor_t executor;

  allocator = rcl_get_default_allocator();
  
  // Initialize shared motor data for M4 communication
  init_shared_motor_data();

  // Create init_options
  rclc_support_init(&support, 0, NULL, &allocator);

  // Create node
  rclc_node_init_default(&node, "stm32_m7_joint_controller", "", &support);

  // Create publisher (for debugging/status)
  rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "stm32_m7_status");

  // Create subscriber for joint commands
  rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
    "/topic_based_joint_commands");

  // Create executor
  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(&executor, &subscriber, &sub_msg, &joint_commands_callback, ON_NEW_DATA);

  pub_msg.data = 0;

  for(;;)
  {
    // Spin executor to process callbacks
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));

    // Publish status message
    rcl_ret_t ret = rcl_publish(&publisher, &pub_msg, NULL);
    if (ret != RCL_RET_OK)
    {
      printf("Error publishing (line %d)\n", __LINE__);
    }

    // Process received joint commands
    process_joint_commands();

    pub_msg.data++;
    osDelay(10);
  }
  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */