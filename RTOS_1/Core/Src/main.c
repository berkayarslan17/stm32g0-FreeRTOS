/*
 * main.c
 *
 * Description:
 *  An example project for running the FreeRTOS on an STM32G031-based board
 * (Cortex-M0) Sends data to the console with tasks which are based on different
 * priorities.
 *
 * Author: Berkay Arslan
 *
 * Project Setup:
 *   Cortex-M0 arch
 *   STM32G031 chip
 */

/*************************************************
 * Definitions
 *************************************************/

#include "main.h"
#include "cmsis_os.h"

UART_HandleTypeDef huart2;

osThreadId defaultTaskHandle;
osThreadId Task2Handle;
osThreadId Task3Handle;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
void StartDefaultTask(void const *argument);
void Task2_init(void const *argument);
void Task3_init(void const *argument);

/*----------------------------------------------------------------------------
 *   Create 'send_data' based functions in order to communicate with UART.
 *---------------------------------------------------------------------------*/

void send_data_deftask(void) {
  uint8_t data[] = "Hello from default task\r\n";
  HAL_UART_Transmit(&huart2, data, sizeof(data), 500);
}

void send_data_task2(void) {
  uint8_t data[] = "Hello from task2\r\n";
  HAL_UART_Transmit(&huart2, data, sizeof(data), 500);
}

void send_data_task3(void) {
  uint8_t data[] = "Hello from task3\r\n";
  HAL_UART_Transmit(&huart2, data, sizeof(data), 500);
}

int main(void) {
  // Initialize the HAL(Hardware Abstraction Layer), GPIO and UART.
  HAL_Init();

  SystemClock_Config();

  MX_GPIO_Init();
  MX_USART2_UART_Init();

  // Define and initialize the tasks.
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  osThreadDef(Task2, Task2_init, osPriorityAboveNormal, 0, 128);
  Task2Handle = osThreadCreate(osThread(Task2), NULL);

  osThreadDef(Task3, Task3_init, osPriorityBelowNormal, 0, 128);
  Task3Handle = osThreadCreate(osThread(Task3), NULL);

  // Start thread execution
  osKernelStart();

  while (1) {
  }
}

void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }
  RCC_ClkInitStruct.ClockType =
      RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
    Error_Handler();
  }
}

static void MX_USART2_UART_Init(void) {

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK) {
    Error_Handler();
  }
}

static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0 | GPIO_PIN_1, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/*----------------------------------------------------------------------------
 *      Thread 1 Send default task data to console
 *---------------------------------------------------------------------------*/

void StartDefaultTask(void const *argument) {
  for (;;) {
    send_data_deftask();
    osDelay(1000); // 1 sec delay
  }
}

/*----------------------------------------------------------------------------
 *      Thread 2 Send task2 data to console
 *---------------------------------------------------------------------------*/

void Task2_init(void const *argument) {
  for (;;) {
    send_data_task2();
    osDelay(1000); // 1 sec delay
  }
}

/*----------------------------------------------------------------------------
 *      Thread 3 Send task3 data to console
 *---------------------------------------------------------------------------*/

void Task3_init(void const *argument) {
  for (;;) {
    send_data_task3();
    osDelay(1000); // 1 sec delay
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {

  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
}

void Error_Handler(void) {}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) {}
#endif
