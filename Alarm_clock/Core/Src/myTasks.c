/*
 * myTasks.c
 *
 *  Created on: May 7, 2024
 *      Author: maciej
 */

#include "myTasks.h"

uint8_t BUTTON_CLICKS = 0;

void state_update_task(void* vParameters)
{
	uint32_t State = 0;

	while(1)
	{
		if(xTaskNotifyWait(0,0,&State,pdMS_TO_TICKS(2000)) == pdPASS)
		{
			//PRINT STATE ON THE LCD
			//xTaskNotify(LCD_HNDL,State,eSetValueWithOverwrite);

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

		if(status_mask & (1 << 0))
		{
			xTaskNotify(LCD_HNDL,long_press,eSetValueWithOverwrite);
		}

		if(status_mask & (1 << 1))
		{
			//REFRESH THE TIMER
			xTimerStart(setup_timer_hndl, 0);
			HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_8);
		}

	}
}

void LCD_task(void* vParameters)
{
	lcd_init();

	uint32_t command = (SET_DDRAM_ADDR)|(LCD_LINE1);
	lcd_send_command(command);
	lcd_send_text("NOT CLICKED");

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
			lcd_send_text("LONG PRESS");
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



