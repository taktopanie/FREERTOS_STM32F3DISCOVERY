/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//#define USE_SEGGER

//#define UART23_bridge

#ifdef USE_SEGGER
	#define DWT_CTRL		(*(volatile uint32_t *) 0xE0001000)
#endif
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
TaskHandle_t SD_card_task;
TaskHandle_t GPS_MSG_check_task;
TaskHandle_t USER_print_task;

//Buffer used to gather GPS data
uint8_t UART_2_buffer [20];

//Buffer used to print USER data on UART3
uint8_t USER_print_buffer [10];

uint8_t rec_buff[500];

volatile GPS_Position_Data_t position;

xSemaphoreHandle xUART_3;
xSemaphoreHandle xUART_2;

xSemaphoreHandle xSD_data;

char send_text [200];
void vApplicationStackOverflowHook( TaskHandle_t xTask, char * pcTaskName );

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
void SD_card_handler (void *pvParameters);
void GPS_MSG_check_handler (void *pvParameters);
void USER_print_handler(void * pvParameters);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void myprintf(const char *fmt, ...) {
  static char buffer[500];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buffer, sizeof(buffer), fmt, args);
  va_end(args);

  int len = strlen(buffer);
  HAL_UART_Transmit(&huart3, (uint8_t*)buffer, len, -1);

  //while UART3 TC not set (transfer not completed)
  uint32_t* wsk = (uint32_t*)0x4000481cUL;
  while(!(*wsk & (1<<6)));

}
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
  Status = xTaskCreate(SD_card_handler , "SD_CARD_task" , 600, "SD_card_proc", 4, &SD_card_task);
  configASSERT(Status == pdPASS);

  Status = xTaskCreate(GPS_MSG_check_handler , "GPS_MSG_check_task" , 500, "GPS MSG checking", 1, &GPS_MSG_check_task);
  configASSERT(Status == pdPASS);

  Status = xTaskCreate(USER_print_handler , "USER print task" , 500, "USER printing", 1, &USER_print_task);
  configASSERT(Status == pdPASS);

  xUART_3 = xSemaphoreCreateMutex();
  configASSERT(xUART_3 != NULL);

  xUART_2 = xSemaphoreCreateMutex();
  configASSERT(xUART_2 != NULL);

  xSD_data = xSemaphoreCreateBinary();
  configASSERT(xSD_data != NULL);
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
#if defined(USE_SEGGER)
		  DWT_CTRL |= (1 << 0);

		  SEGGER_SYSVIEW_Conf();

		  SEGGER_SYSVIEW_Start();
#endif
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI2_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */

  HAL_UART_Receive_IT(&huart2, UART_2_buffer, 1);
  HAL_UART_Receive_IT(&huart3, USER_print_buffer, 1);

  // turn ON working LED and send hello text
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, ENABLE);
  myprintf("\r\n########################################################\n"
		  	  "#                                                      #\n"
		  	  "# Program has been started...                          #\n"
		  	  "# The logs will be automatically collected             #\n"
		  	  "#                                                      #\n"
		  	  "# Press the USER button to unmount the SD CARD!!!      #\n"
		  	  "#                                                      #\n"
		  	  "########################################################\n");

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
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : DRDY_Pin MEMS_INT3_Pin MEMS_INT4_Pin MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = DRDY_Pin|MEMS_INT3_Pin|MEMS_INT4_Pin|MEMS_INT2_Pin;
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

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI1_SCK_Pin SPI1_MISO_Pin SPI1_MISOA7_Pin */
  GPIO_InitStruct.Pin = SPI1_SCK_Pin|SPI1_MISO_Pin|SPI1_MISOA7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SD_CS_Pin */
  GPIO_InitStruct.Pin = SD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SD_CS_GPIO_Port, &GPIO_InitStruct);

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

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void SD_card_handler (void *pvParameters)
{
	/*
	 * Private variables
	 */

	//received position calculated
	GPS_Position_Data_t * position_addr = 0;

	//address received throught notification
	uint32_t rec_addr = 0;

	//buffer to write the data to SD CARD
	BYTE writebuff[100];		//TODO: BUFFER SIZE WILL BE ANALYZED IN THE FUTURE

	//SDcard mounting status
	uint8_t SD_mount_status = 0;

	//variables for FatFs
	FATFS FatFs; 				//Fatfs handle
	FIL fil; 					//File handle
	FRESULT fres; 				//Result after operations


	/*
	 * Code
	 */

	xSemaphoreTake(xUART_3,0);
	myprintf("\rSD card INITIALIZE \r");
	vTaskDelay(pdMS_TO_TICKS(1000)); //a short delay is important to let the SD card settle
	xSemaphoreGive(xUART_3);

	portENTER_CRITICAL();
	//Open the file system
	SD_mount_status = f_mount(&FatFs, "", 1); //1=mount now

	if (SD_mount_status != FR_OK) {
		myprintf("f_mount error (%i)\r\n", SD_mount_status);
	}else
	{
		//Try to create "LOG.txt" file
		fres = f_open(&fil, "position.log", FA_WRITE | FA_OPEN_ALWAYS | FA_CREATE_ALWAYS);

		if(fres == FR_OK) {
			myprintf("Logfile was created\r\n");

			//SD MOUNT LED INDICATORS (GREEN LEDS)
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, ENABLE);
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, ENABLE);
		} else {
			myprintf("f_open error (%i)\r\n", fres);
		}
		//close file
		f_close(&fil);

	}
	portEXIT_CRITICAL();

	while(1)
	{
		if(xTaskNotifyWait(0, 0, &rec_addr, portMAX_DELAY) == pdPASS)
		{
			position_addr = (GPS_Position_Data_t *)rec_addr;


			if(SD_mount_status == FR_OK)
			{
				//portENTER_CRITICAL();			<<<<<<<<<<<< THIS CRITICAL SECTION CAUSING THE CRASH

				fres = f_open(&fil, "position.log", FA_WRITE | FA_OPEN_ALWAYS );
				if (fres == FR_OK)
				{
					//APPEND
					f_lseek(&fil, fil.fsize);

					sprintf((char*)writebuff, "%d.%ld,%d.%ld,%02d:%02d:%02d\n", position_addr->Latitude_deg, \
						position_addr->Latitude_minINdegrees, position_addr->Longtitude_deg, \
						position_addr->Longtitude_minINdegrees, position_addr->Time.hours, \
						position_addr->Time.minutes, position_addr->Time.seconds);

					 uint8_t length = strlen((char*)writebuff);
					 UINT bytesWrote;

					 fres = f_write(&fil, writebuff, length, &bytesWrote);
					 if(fres == FR_OK) {
						 if(xSemaphoreTake(xUART_3,0) == pdPASS)
						 {
							myprintf("Logfile updated\r\n");
							xSemaphoreGive(xUART_3);
						 }

					 } else {
						 if(xSemaphoreTake(xUART_3,0) == pdPASS)
						 {
								myprintf("f_write error (%i)\r\n");
								xSemaphoreGive(xUART_3);
						 }
					 }
					 f_close(&fil);
				} else {
					 if(xSemaphoreTake(xUART_3,0) == pdPASS)
					 {
						 myprintf("f_open error (%i)\r\n", fres);
						xSemaphoreGive(xUART_3);
					 }
				 }
				//flush the buffor
				for(int i = 0; i < 30; i++)
				{
					writebuff[i] = '\0';
				}

				// portEXIT_CRITICAL();			<<<<<<<<<<<< THIS CRITICAL SECTION CAUSING THE CRASH
			}
			//DATA MAY BE FREED
			xSemaphoreGive(xSD_data);
		}
	}
}

void GPS_MSG_check_handler (void *pvParameters)
{
	/*
	 * Private variables
	 */

	uint8_t buffor_size = 0;

	/*
	 * Code
	 */

	while(1)
	{
		//wait for notify from UART receiver
		xTaskNotifyWait(0, 0, (uint32_t *)&buffor_size, portMAX_DELAY);

		//TODO: this is not working, IRQ's are working
		portDISABLE_INTERRUPTS();

		//checks if pending message is position info
		position = GPS_get_position(rec_buff, buffor_size);
		if(position.Latitude_deg != 0)
		{
			xTaskNotify(USER_print_task, (uint32_t)&position, eSetValueWithOverwrite);

			//wait until data will be freed
			xSemaphoreTake(xSD_data, portMAX_DELAY);
		}
		//flush the buffer
		for(int i = 0; i < buffor_size; i++)
		{
			rec_buff[i] = '\0';
		}
		portENABLE_INTERRUPTS();
	}
}

void USER_print_handler (void *pvParameters)
{
	GPS_Position_Data_t * position_addr = 0;
	uint32_t rec_addr = 0;

	char msg [300];

	while(1)
	{
		if(xSemaphoreTake(xUART_3,0)==pdPASS)
		{
			//notify the user that program is running
			myprintf("RUNNING\n");
			xSemaphoreGive(xUART_3);
		}

		if(xTaskNotifyWait(0, 0, (uint32_t*)&rec_addr, pdMS_TO_TICKS(1000)) == pdPASS)
		{
			position_addr = (GPS_Position_Data_t *)rec_addr;

			//notify the user that data has been received
			sprintf(msg,"Position: %d.%ld %d.%ld", position_addr->Latitude_deg, position_addr->Latitude_minINdegrees, \
					position_addr->Longtitude_deg, position_addr->Longtitude_minINdegrees);
			if(xSemaphoreTake(xUART_3,0)==pdPASS)
			{
				myprintf(msg);
				xSemaphoreGive(xUART_3);
			}

			#if defined(TIME_ZONE)
				sprintf(msg," TIME: %02d:%02d:%02d\n", (position_addr->Time.hours + TIME_ZONE), position_addr->Time.minutes, position_addr->Time.seconds);
			#else
				sprintf(msg," TIME UTC: %02d:%02d:%02d\n", (position_addr->Time.hours), position_addr->Time.minutes, position_addr->Time.seconds);
			#endif
				if(xSemaphoreTake(xUART_3,0)==pdPASS)
				{
					myprintf(msg);
					xSemaphoreGive(xUART_3);
				}

			//notify SD_card task to save the data and toggle the LED
			xTaskNotify(SD_card_task,rec_addr,eSetValueWithOverwrite);
			HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_9);
		}
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

#if defined(USE_SEGGER)
	SEGGER_SYSVIEW_RecordEnterISR();
#endif

	static uint8_t rec_status = 0;

	if(huart == &huart3)
	{

		HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_14);

#if defined(UART23_bridge)
		if(xSemaphoreTakeFromISR(xUART_2, 0) == pdPASS)
		{
			//SEND FROM UART2 => UART3
			HAL_UART_Transmit(&huart2, USER_print_buffer, 1, 0); // 0 delay - very important!!
			xSemaphoreGiveFromISR(xUART_2, 0);
		}
#endif
		HAL_UART_Receive_IT(&huart3, USER_print_buffer, 1);

	}
	if(huart == &huart2)
	{
		HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_13);

#if defined(UART23_bridge)
		if(xSemaphoreTakeFromISR(xUART_3, 0) == pdPASS){
			//SEND FROM UART2 => UART3
			HAL_UART_Transmit(&huart3, UART_2_buffer, 1, 0); // 0 delay - very important!!
			xSemaphoreGiveFromISR(xUART_3, 0);
		}
#endif
		//if message will be analyzed
		if(rec_status)
		{
			rec_buff[rec_status-1] = UART_2_buffer[0];
			UART_2_buffer[0] = '\0';
			if(rec_buff[rec_status-1] == '\n')
			{
				//notify the MSG checking task
				xTaskNotifyFromISR(GPS_MSG_check_task, (rec_status-1), eSetValueWithOverwrite, 0);
				rec_status = 0;
			}else{
				rec_status++;
			}

		}else
		{
			//Start of the phrase, start recording
			if(UART_2_buffer[0] == '$'){
			rec_status = 1;
			}
		}
		HAL_UART_Receive_IT(&huart2, UART_2_buffer, 1);



	}

#if defined(USE_SEGGER)
	SEGGER_SYSVIEW_RecordExitISR();
#endif

}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PIN_0)
	{
		//SUSPEND THE SD CARD TASK and unmount the SD card
		f_mount(NULL, "", 0);

		//TURN OFF SD CARD LEDS
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, DISABLE);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, DISABLE);

		//TURN OFF STATUS LED
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, DISABLE);

		myprintf("Logging has been stopped, SDCARD can be removed now...\n");

		vTaskSuspend(SD_card_task);
	}
}

void vApplicationStackOverflowHook( TaskHandle_t xTask, char * pcTaskName )
{
	myprintf("Stack overflowed by: %s \r\n", pcTaskName);
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
