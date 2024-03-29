/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include"FreeRTOS.h"
#include "task.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define LED_GREEN_1 	GPIO_PIN_15
#define LED_GREEN_2 	GPIO_PIN_11
#define LED_ORANGE_1 	GPIO_PIN_14
#define LED_ORANGE_2 	GPIO_PIN_10
#define LED_BLUE_1 		GPIO_PIN_12
#define LED_BLUE_2 		GPIO_PIN_8
#define LED_RED_1		GPIO_PIN_13
#define LED_RED_2		GPIO_PIN_9
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
#define DWT_CTRL		(*(volatile uint32_t *) 0xE0001000)
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
static void toggle_1_handler(void* parameters);
static void toggle_2_handler(void* parameters);
static void toggle_3_handler(void* parameters);
static void toggle_4_handler(void* parameters);
static void ALL_LEDS_ON_handler(void* parameters);
static void button_press_handler(void* parameters);


TaskHandle_t volatile next_task_handle;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
TaskHandle_t toggle_1_handle;
TaskHandle_t toggle_2_handle;
TaskHandle_t toggle_3_handle;
TaskHandle_t toggle_4_handle;
TaskHandle_t LEDS_ON_handle;
TaskHandle_t BTN_task_handle;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	BaseType_t Status;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  /* USER CODE BEGIN 2 */
  DWT_CTRL |= (1 << 0);

  //////////////TODO: vTaskSuspend instead of deleting tasks

  SEGGER_SYSVIEW_Conf();

  SEGGER_SYSVIEW_Start();

  Status = xTaskCreate(toggle_1_handler, "Toggle 1 LED", 200, "1 LED toggled", 1, &toggle_1_handle );
  configASSERT(Status == pdPASS);

  next_task_handle = toggle_1_handle;

  Status = xTaskCreate(toggle_2_handler, "Toggle 2 LED", 200, "2 LED toggled", 1, &toggle_2_handle );
  configASSERT(Status == pdPASS);

  Status = xTaskCreate(toggle_3_handler, "Toggle 3 LED", 200, "3 LED toggled", 1, &toggle_3_handle );
  configASSERT(Status == pdPASS);

  Status = xTaskCreate(toggle_4_handler, "Toggle 4 LED", 200, "4 LED toggled", 1, &toggle_4_handle );
  configASSERT(Status == pdPASS);

  Status = xTaskCreate(ALL_LEDS_ON_handler, "ALL LEDS ON", 200, "all leds on", 1, &LEDS_ON_handle );
  configASSERT(Status == pdPASS);

  Status = xTaskCreate(button_press_handler, "Button task", 200, "Button pressed", 2, &BTN_task_handle );
  configASSERT(Status == pdPASS);

  //START SCHEDULER
  vTaskSuspend(LEDS_ON_handle);
  vTaskStartScheduler();

  /* USER CODE END 2 */

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, CS_I2C_SPI_Pin|LD4_Pin|LD3_Pin|LD5_Pin
                          |LD7_Pin|LD9_Pin|LD10_Pin|LD8_Pin
                          |LD6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : DRDY_Pin MEMS_INT3_Pin MEMS_INT4_Pin MEMS_INT1_Pin
                           MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = DRDY_Pin|MEMS_INT3_Pin|MEMS_INT4_Pin|MEMS_INT1_Pin
                          |MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : CS_I2C_SPI_Pin LD4_Pin LD3_Pin LD5_Pin
                           LD7_Pin LD9_Pin LD10_Pin LD8_Pin
                           LD6_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin|LD4_Pin|LD3_Pin|LD5_Pin
                          |LD7_Pin|LD9_Pin|LD10_Pin|LD8_Pin
                          |LD6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI1_SCK_Pin SPI1_MISO_Pin SPI1_MISOA7_Pin */
  GPIO_InitStruct.Pin = SPI1_SCK_Pin|SPI1_MISO_Pin|SPI1_MISOA7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : DM_Pin DP_Pin */
  GPIO_InitStruct.Pin = DM_Pin|DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF14_USB;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : I2C1_SCL_Pin I2C1_SDA_Pin */
  GPIO_InitStruct.Pin = I2C1_SCL_Pin|I2C1_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
static void toggle_1_handler(void* parameters){

	BaseType_t status;
	while(1){
		SEGGER_SYSVIEW_PrintfTarget("Toggling 1 LED");
		HAL_GPIO_TogglePin(GPIOE, LED_GREEN_1);
		HAL_GPIO_TogglePin(GPIOE, LED_GREEN_2);
		status = xTaskNotifyWait(0,0,NULL,pdMS_TO_TICKS(1000));

		if(status == pdTRUE){
			//the button was pressed
			taskENTER_CRITICAL();
			next_task_handle = toggle_2_handle;
			taskEXIT_CRITICAL();

			eTaskState state = eTaskGetState(toggle_1_handle);

			if(state == eReady || state == eRunning){
				HAL_GPIO_WritePin(GPIOE, LED_GREEN_1, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOE, LED_GREEN_2, GPIO_PIN_SET);
				vTaskSuspend(NULL);

			}else{
			}
		}
	}
}

static void toggle_2_handler(void* parameters){

	BaseType_t status;
	while(1){
		SEGGER_SYSVIEW_PrintfTarget("Toggling 2 LED");
		HAL_GPIO_TogglePin(GPIOE, LED_ORANGE_1);
		HAL_GPIO_TogglePin(GPIOE, LED_ORANGE_2);
		status = xTaskNotifyWait(0,0,NULL,pdMS_TO_TICKS(500));

		if(status == pdTRUE){
			//the button was pressed
			taskENTER_CRITICAL();
			next_task_handle = toggle_3_handle;
			taskEXIT_CRITICAL();

			eTaskState state = eTaskGetState(toggle_2_handle);

			if(state == eReady || state == eRunning){
				HAL_GPIO_WritePin(GPIOE, LED_ORANGE_1, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOE, LED_ORANGE_2, GPIO_PIN_SET);
				vTaskSuspend(NULL);

			}else{
			}
		}
	}
}

static void toggle_3_handler(void* parameters){

	BaseType_t status;
	while(1){
		SEGGER_SYSVIEW_PrintfTarget("Toggling 3 LED");
		HAL_GPIO_TogglePin(GPIOE, LED_RED_1);
		HAL_GPIO_TogglePin(GPIOE, LED_RED_2);
		status = xTaskNotifyWait(0,0,NULL,pdMS_TO_TICKS(250));

		if(status == pdTRUE){
			//the button was pressed
			taskENTER_CRITICAL();
			next_task_handle = toggle_4_handle;
			taskEXIT_CRITICAL();

			eTaskState state = eTaskGetState(toggle_3_handle);

			if(state == eReady || state == eRunning){
				HAL_GPIO_WritePin(GPIOE, LED_RED_1, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOE, LED_RED_2, GPIO_PIN_SET);
				vTaskSuspend(NULL);

			}else{
			}
		}
	}
}

static void toggle_4_handler(void* parameters){

	BaseType_t status;
	while(1){
		SEGGER_SYSVIEW_PrintfTarget("Toggling 4 LED");
		HAL_GPIO_TogglePin(GPIOE, LED_BLUE_1);
		HAL_GPIO_TogglePin(GPIOE, LED_BLUE_2);

		status = xTaskNotifyWait(0,0,NULL,pdMS_TO_TICKS(125));

		if(status == pdTRUE){
			//the button was pressed
			taskENTER_CRITICAL();
			next_task_handle = LEDS_ON_handle;
			taskEXIT_CRITICAL();

			eTaskState state = eTaskGetState(toggle_4_handle);

			if(state == eReady || state == eRunning){
				HAL_GPIO_WritePin(GPIOE, LED_BLUE_1, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOE, LED_BLUE_2, GPIO_PIN_SET);
				vTaskResume(LEDS_ON_handle);
				vTaskSuspend(NULL);
			}else{
			}
		}
	}
}

static void ALL_LEDS_ON_handler(void* parameters){

	BaseType_t status;
	while(1){
		SEGGER_SYSVIEW_PrintfTarget("LEDS ON");
		status = xTaskNotifyWait(0,0,NULL,pdMS_TO_TICKS(500));

		if(status == pdTRUE){
			//the button was pressed
			taskENTER_CRITICAL();
			next_task_handle = toggle_1_handle;
			taskEXIT_CRITICAL();

				vTaskResume(toggle_1_handle);
				vTaskResume(toggle_2_handle);
				vTaskResume(toggle_3_handle);
				vTaskResume(toggle_4_handle);
				vTaskSuspend(NULL);
		}
	}
}

static void button_press_handler(void* parameters){

	uint8_t btn_read = 0;
	uint8_t prev_state = 0;

	while(1){
		btn_read = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);

		if(btn_read){
			if(!prev_state){
				xTaskNotify(next_task_handle, 0, eNoAction);
			}
		}
		prev_state = btn_read;
		vTaskDelay(pdMS_TO_TICKS(10));
	}

}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
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

#ifdef  USE_FULL_ASSERT
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
