/*
 * led_effect.c
 *
 *  Created on: Oct 13, 2023
 *      Author: taktopanie
 */


#include "main.h"


void led_effect_stop(void){

	for(int i =0; i<4; i++){
		xTimerStop(handle_led_timer[i], portMAX_DELAY);
	}
}

void led_effect(int e){

	led_effect_stop();
	xTimerStart(handle_led_timer[e-1], portMAX_DELAY);

}

void turn_off_all_leds(void)
{
	HAL_GPIO_WritePin(LD10_GPIO_Port, LD10_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LD9_GPIO_Port, LD9_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LD8_GPIO_Port, LD8_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LD7_GPIO_Port, LD7_Pin,GPIO_PIN_RESET);
}


void turn_on_all_leds(void)
{
	HAL_GPIO_WritePin(LD10_GPIO_Port, LD10_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(LD9_GPIO_Port, LD9_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(LD8_GPIO_Port, LD8_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(LD7_GPIO_Port, LD7_Pin,GPIO_PIN_SET);
}

void turn_on_odd_leds(void)
{
	HAL_GPIO_WritePin(LD10_GPIO_Port, LD10_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(LD9_GPIO_Port, LD9_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LD8_GPIO_Port, LD8_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(LD7_GPIO_Port, LD7_Pin,GPIO_PIN_RESET);
}


void turn_on_even_leds(void)
{
	HAL_GPIO_WritePin(LD10_GPIO_Port, LD10_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LD9_GPIO_Port, LD9_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(LD8_GPIO_Port, LD8_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LD7_GPIO_Port, LD7_Pin,GPIO_PIN_SET);
}

void LED_control( int value )
{
  for(int i = 0 ; i < 4 ; i++)
	  HAL_GPIO_WritePin(LD10_GPIO_Port, (LD7_Pin << i), ((value >> i)& 0x1));
}


void LED_effect1(void)
{
	static int flag = 1;
	(flag ^= 1) ? turn_off_all_leds() : turn_on_all_leds();
}


void LED_effect2(void)
{
	static int flag = 1;
	(flag ^= 1) ? turn_on_even_leds() : turn_on_odd_leds();
}

void LED_effect3(void)
{
	static int i = 0;
	LED_control( 0x1 << (i++ % 4) );
}


void LED_effect4(void)
{
	static int i = 0;
	LED_control( 0x08 >> (i++ % 4) );
}
