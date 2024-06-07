/*
 * myTasks.c
 *
 *  Created on: May 7, 2024
 *      Author: maciej
 */

#include "myTasks.h"
#include "stdio.h"
#include "DS3231.h"

uint8_t BUTTON_CLICKS = 0;


struct global_time
{
	uint16_t hours;
	uint16_t minutes;
	uint16_t seconds;

};

struct global_time TIME = {18,00,00};

void state_update_task(void* vParameters)
{
	uint32_t State = 0;

	while(1)
	{
		if(xTaskNotifyWait(0,0,&State,pdMS_TO_TICKS(2000)) == pdPASS)
		{

			//ANY OPTION DIFFERENT THAN NO CLICK
			if(State > 0)
			{
				xTimerStart(setup_timer_hndl, 0);
				push_state = State;

				switch(State)
					{
					case short_click:
						xTaskNotify(SHORT_CLICK_HNDL, 1, eSetBits);
						break;
					case double_click:
						xTaskNotify(DOUBLE_CLICK_HNDL, 1, eSetBits);
						break;
					case long_press:
						xTaskNotify(LONG_PRESS_HNDL, 1, eSetBits);
						break;
					}

			}
		}
		HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_13);
	}

}

void DOUBLE_CLICK_task(void* vParameters)
{
	while(1)
	{
		xTaskNotifyWait(0,0,NULL,portMAX_DELAY);
		xTaskNotify(LCD_HNDL,double_click,eSetValueWithOverwrite);
	}
}

void SHORT_CLICK_task(void* vParameters)
{

	while(1)
	{
		xTaskNotifyWait(0,0,NULL,portMAX_DELAY);
		xTaskNotify(LCD_HNDL,short_click,eSetValueWithOverwrite);

	}
}

void LONG_PRESS_task(void* vParameters)
{
	uint32_t status_mask = 0;
	while(1)
	{
		xTaskNotifyWait(0,0xF,&status_mask,portMAX_DELAY);

		//STATUS LONG_PRESS TASK INIT
		if(status_mask & (1 << 0))
		{
			xTaskNotify(LCD_HNDL,long_press,eSetValueWithOverwrite);
		}

		//STATUS LONG_PRESS BUTTON PRESSED
		if(status_mask & (1 << 1))
		{
			//BUTTON DEBOUNCING
			vTaskDelay(pdMS_TO_TICKS(20));
			//REFRESH THE TIMER
			xTimerStart(setup_timer_hndl, 0);
			_CLOCK_second_increment();
			xTaskNotify(LCD_HNDL,long_press,eSetValueWithOverwrite);

			//TODO:
			uint32_t command = (SET_DDRAM_ADDR)|(LCD_LINE1+0x7);
			lcd_send_command(command);

			HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_8);
			while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) == GPIO_PIN_RESET);

		}

	}
}

void LCD_task(void* vParameters)
{
	lcd_init();
	char message [16];

	uint32_t command = (SET_DDRAM_ADDR)|(LCD_LINE1);

	lcd_send_command(command);

	lcd_send_text("Press the button");

	uint32_t State = 0;

	while(1)
	{
		xTaskNotifyWait(0,0,&State,portMAX_DELAY);
		if(State == short_click)
		{
			lcd_clear();
			command = (SET_DDRAM_ADDR)|(LCD_LINE1);
			lcd_send_command(command);
			lcd_send_text("SHORT CLICK");
		}else if(State == long_press)
		{
			lcd_clear();
			command = (SET_DDRAM_ADDR)|(LCD_LINE1);
			lcd_send_command(command);

			//itoa(global_value, message, 10);

			//TODO: FORMAT WILL BE CHECKED
			sprintf(message, "%02d:%02d:%02d", TIME.hours, TIME.minutes, TIME.seconds);

			lcd_send_text(message);
		}else if(State == double_click)
		{
			lcd_clear();
			command = (SET_DDRAM_ADDR)|(LCD_LINE1);
			lcd_send_command(command);
			lcd_send_text("DOUBLE CLICK");
		}
		else if(State == not_clicked)
		{
			lcd_clear();
			command = (SET_DDRAM_ADDR)|(LCD_LINE1);
			lcd_send_command(command);
			lcd_send_text("NOT CLICKED");
		}

	}
}

void DS3231_task(void* vParameters)
{
	DS3231_Time_t timer_time = {0,0,0,0,0,0,0};
	while(1)
	{
		xTaskNotifyWait(0,0,NULL, portMAX_DELAY);

		//RETREIVE THE VALUES FROM RTC
		timer_time = DS3231_get_time(&hi2c1);

		//COPY THE VALUES TO GLOBAL VARIABLE
		TIME.hours = timer_time.time_hr;
		TIME.minutes = timer_time.time_min;
		TIME.seconds = timer_time.time_sec;

	}
}

void CLOCK_TICK_task(void* vParameters)
{
	TickType_t xLastWakeTime;
	// Initialise the xLastWakeTime variable with the current time.
	xLastWakeTime = xTaskGetTickCount();

	while(1)
	{
		xTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1000));
		xTaskNotify(DS3231_HNDL, 0, eNoAction);
		_CLOCK_second_increment();
	}
}

void _CLOCK_second_increment(void)
{
	TIME.seconds++;

	if(TIME.seconds == 60)
	{
		TIME.minutes++;
		TIME.seconds = 0;

		if(TIME.minutes == 60)
		{
			TIME.hours++;
			TIME.minutes = 0;
			if(TIME.hours == 24)
			{
				TIME.hours = 0;
			}

		}
	}
}


void setup_timer_expiry(TimerHandle_t xTimer)
{
	push_state = not_clicked;

	//TURN OFF ALL LEDS
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, DISABLE);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, DISABLE);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, DISABLE);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, DISABLE);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, DISABLE);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, DISABLE);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, DISABLE);

	xTaskNotify(LCD_HNDL,push_state,eSetValueWithOverwrite);
}



