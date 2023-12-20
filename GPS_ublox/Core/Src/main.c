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
#include"FreeRTOS.h"
#include "task.h"
#include "my_GPS.h"
#include "semphr.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/*
 * UART2 is used for GPS module
 * 	PA2 - TX
 * 	PA3 - RX
 * UART3 is used for user
 *  PB10 - TX
 *  PB11 - RX
 *
 */
/*
 * DEFINE IF YOU WANT TO BRIDGE THESE TWO UARTS -- MOSTLY USED DURING CONFIGURATION GPS RECEIVER
 */
//#define UART23_bridge
/*
 * DEFINE YOUR TIMEZONE HERE, IF NOT IT WILL PRINT UTC TIME
 */
#define TIME_ZONE 1
#define USE_SEGGER

#ifdef USE_SEGGER
	#define DWT_CTRL		(*(volatile uint32_t *) 0xE0001000)
#endif

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
TaskHandle_t MSG_check_task;
TaskHandle_t UART3_task;

uint8_t UART_2_buffor [10];
uint8_t UART_3_buffor [10];

uint8_t rec_buff[500];

volatile GPS_Position_Data_t position;

xSemaphoreHandle xUART_3;
xSemaphoreHandle xUART_2;

char send_text [200];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
void printmsg(char *msg);

void MSG_check_handler(void * pvParameters);
void UART_3_handler(void * pvParameters);


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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

#if defined(USE_SEGGER)
		  DWT_CTRL |= (1 << 0);


		  SEGGER_SYSVIEW_Conf();

		  SEGGER_SYSVIEW_Start();
#endif

  Status = xTaskCreate(MSG_check_handler , "MSG_check_task" , 200, "MSG checking", 1, &MSG_check_task);
  configASSERT(Status == pdPASS);

  Status = xTaskCreate(UART_3_handler , "UART3_task" , 200, "UART3 task running", 1, &UART3_task);
  configASSERT(Status == pdPASS);

  xUART_3 = xSemaphoreCreateMutex();
  xUART_2 = xSemaphoreCreateMutex();

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  sprintf(send_text, "\n###############################\n#                             #\n"\
		  "#      Program is running     #\n#   Waiting for GSP data...   #\n"\
		  "#                             #\n###############################\n\n");

  //send welcome text
  printmsg(send_text);

  HAL_UART_Receive_IT(&huart2, UART_2_buffor, 1);
  HAL_UART_Receive_IT(&huart3, UART_3_buffor, 1);

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_USART3;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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

void printmsg(char *msg){

		xSemaphoreTake(xUART_3, portMAX_DELAY);

		while ( __HAL_UART_GET_FLAG(&huart3, UART_FLAG_TXE) != SET);

		HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

		while ( __HAL_UART_GET_FLAG(&huart3, UART_FLAG_TC) != SET);

		xSemaphoreGive(xUART_3);


}

void MSG_check_handler(void * pvParameters)
{
	uint8_t buffor_size = 0;
	while(1)
	{
		//wait for notify from UART receiver
		xTaskNotifyWait(0, 0, (uint32_t *)&buffor_size, portMAX_DELAY);

		//checks if pending message is position info
		position = GPS_get_position(rec_buff, buffor_size);
		if(position.Latitude_deg != 0)
		{
			xTaskNotify(UART3_task, (uint32_t)&position, eSetValueWithOverwrite);
		}
		//flush the buffor
		for(int i = 0; i < 100; i++)
		{
			rec_buff[i] = '\0';
		}
	}
}

void UART_3_handler(void * pvParameters)
{
	GPS_Position_Data_t * position_addr = 0;
	uint32_t rec_addr = 0;
	  // turn ON working LED and send hello text
	  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, ENABLE);

	  char msg [300];

	while(1)
	{
		if(xTaskNotifyWait(0, 0, (uint32_t*)&rec_addr, portMAX_DELAY) == pdPASS)
		{
			position_addr = (GPS_Position_Data_t *)rec_addr;

			sprintf(msg,"Position: %d.%ld %d.%ld", position_addr->Latitude_deg, position_addr->Latitude_minINdegrees, \
					position_addr->Longtitude_deg, position_addr->Longtitude_minINdegrees);
			printmsg(msg);

			#if defined(TIME_ZONE)
				sprintf(msg," TIME: %02d:%02d:%02d\n", (position_addr->Time.hours + TIME_ZONE), position_addr->Time.minutes, position_addr->Time.seconds);
			#else
				sprintf(msg," TIME UTC: %02d:%02d:%02d\n", (position_addr->Time.hours), position_addr->Time.minutes, position_addr->Time.seconds);
			#endif
				printmsg(msg);
			HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_9);
		}
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	static uint8_t rec_status = 0;

	if(huart == &huart3)
	{

		HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_14);

#if defined(UART23_bridge)
		if(xSemaphoreTakeFromISR(xUART_2, 0) == pdPASS)
		{
			//SEND FROM UART2 => UART3
			HAL_UART_Transmit(&huart2, UART_3_buffor, 1, 0); // 0 delay - very important!!
			xSemaphoreGiveFromISR(xUART_2, 0);
		}
#endif
		HAL_UART_Receive_IT(&huart3, UART_3_buffor, 1);

	}
	if(huart == &huart2)
	{
		HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_13);

#if defined(UART23_bridge)
		if(xSemaphoreTakeFromISR(xUART_3, 0) == pdPASS){
			//SEND FROM UART2 => UART3
			HAL_UART_Transmit(&huart3, UART_2_buffor, 1, 0); // 0 delay - very important!!
			xSemaphoreGiveFromISR(xUART_3, 0);
		}
#endif
		//if message will be analyzed
		if(rec_status)
		{
			rec_buff[rec_status-1] = UART_2_buffor[0];
			if(rec_buff[rec_status-1] == '\n')
			{
				//notify the MSG checking task
				xTaskNotifyFromISR(MSG_check_task, (rec_status-1), eSetValueWithOverwrite, 0);
				rec_status = 0;
			}else{
				rec_status++;
			}

		}
		//Start of the phrase, start recording
		if(UART_2_buffor[0] == '$'){
			rec_status = 1;
		}

		HAL_UART_Receive_IT(&huart2, UART_2_buffor, 1);



	}

}

/* USER CODE END 4 */

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
