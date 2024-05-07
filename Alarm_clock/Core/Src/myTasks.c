/*
 * myTasks.c
 *
 *  Created on: May 7, 2024
 *      Author: maciej
 */

#include "myTasks.h"

void state_update_task(void* vParameters)
{
	uint32_t State = 0;

	while(1)
	{
		if(xTaskNotifyWait(0,0,&State,pdMS_TO_TICKS(2000)) == pdPASS)
		{
			//PRINT STATE ON THE LCD
			xTaskNotify(LCD_hndl,State,eSetValueWithOverwrite);

			//ANY OPTION DIFFERENT THAN NO CLICK
			if(State > 0)
			{
				xTimerStart(setup_timer_hndl, 0);
			}
		}
		HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_13);
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
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, DISABLE);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, DISABLE);

	xTaskNotify(LCD_hndl,push_state,eSetValueWithOverwrite);
}
