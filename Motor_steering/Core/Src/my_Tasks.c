/*
 * my_Tasks.c
 *
 *  Created on: Feb 23, 2024
 *      Author: maciej
 */

#include "main.h"

uint16_t _meas = 0;
void vMeasurements (void* pvParameters)
{

	uint16_t _old_meas_servo = 0;
	uint16_t _old_meas_stepper = 0;

	//measure threshold to deal with ADC fluctuations (servo and stepper has different sensitivity)
	uint8_t _meas_threshold_servo = 10;
	uint8_t _meas_threshold_stepper = 30;

	uint32_t stepper_mask = 0;

	//TIMER WHICH WILL BE USED TO TRIGGER THE ADC MEASUREMENTS
	HAL_TIM_OC_Start(&htim4, TIM_CHANNEL_4);

	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&_meas, 1);


	while(1)
	{
		//IF VALUE CHANGED
		if(_old_meas_servo >= (_meas + _meas_threshold_servo) || _old_meas_servo < (_meas - _meas_threshold_servo))
		{
			xTaskNotify(motor_servo, _meas, eSetValueWithOverwrite);
			_old_meas_servo = _meas;
		}

		if(_old_meas_stepper >= (_meas + _meas_threshold_stepper) || _old_meas_stepper < (_meas - _meas_threshold_stepper))
		{
			/*
			 * STEPPER MOTOR SETTING UP
			 *
			 *
			 * 3 speeds on the left/right + stop on the middle 4094 / 7 == 585 per mode
			 *
			 */

			//LEFT TURN
			if(_meas < 585){
				//speed 1 left,
				stepper_mask = (0<<0) | (1<<3);
			}
			else if (_meas >=585 && _meas <1170)
			{
				stepper_mask = (0<<0) | (1<<2);
				//speed 2 left
			}
			else if (_meas >=1170 && _meas <1755)
			{
				stepper_mask = (0<<0) | (1<<1);
				//speed 3 left
			}

			//IDLE MODE
			else if (_meas >=1850 && _meas <2200)
			{
				//idle motor
				stepper_mask = 0;
			}

			//RIGHT TURN
			else if (_meas >=2340 && _meas <2925)
			{
				//speed 1 right
				stepper_mask = (1<<0) | (1<<1);
			}
			else if (_meas >=2925 && _meas <3510)
			{
				//speed 2 right
				stepper_mask = (1<<0) | (1<<2);
			}
			else if (_meas >=3510 && _meas <4095)
			{
				//speed 3 right
				stepper_mask = (1<<0) | (1<<3);
			}

			xTaskNotify(motor_stepper, stepper_mask, eSetBits);
			_old_meas_stepper = _meas;
		}

		vTaskDelay(pdMS_TO_TICKS(5));
	}
}

//After setting motor via Timer this task only save the actual motor speed
void vMotorControl (void* pvParameters)
{
	//IF HARDWARE TIMER WILL BE USED
	//HAL_TIM_Base_Start_IT(&htim3);
	uint32_t rec_time;
	uint8_t gear_old = 0;

	while(1)
	{
		//save the motor speed
		if(xTaskNotifyWait(0xFF,0,&rec_time,portMAX_DELAY) == pdPASS)
		{
			//IDLE OR NOT
			if(rec_time)
			{
				//DIRECTION
				if(rec_time & (1<<0))
				{
					//RIGHT
					MOTOR_DIR = 1;
				}else
				{
					//LEFT
					MOTOR_DIR = 0;
				}

				if(rec_time & (1<<1))
				{
					//GEAR 1
					if(gear_old != 1){
					xTimerChangePeriodFromISR(Timer_1, pdMS_TO_TICKS(1000), 0);
					gear_old = 1;
					}
				}
				else if(rec_time & (1<<2))
				{
					//GEAR 2
					if(gear_old != 2){
					xTimerChangePeriodFromISR(Timer_1, pdMS_TO_TICKS(300), 0);
					gear_old = 2;
					}
				}
				else if(rec_time & (1<<3))
				{
					//GEAR 3
					if(gear_old != 3){
					xTimerChangePeriodFromISR(Timer_1, pdMS_TO_TICKS(10), 0);
					gear_old = 3;
					}
				}

			}
			else
			{
				//MOTOR IDLE
				gear_old = 0;
				MOTOR_set_position_ULN2003(99);
				xTimerChangePeriodFromISR(Timer_1, portMAX_DELAY, 0);
			}
			//rec_time is the actual motor speed
		}
	}
}

void vServoControl (void* pvParameters)
{

	uint8_t _servo_position = 0;
	uint16_t _adc_meas = 0;

	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

	//set initial position
	SERVO_set(&htim2, 0);

	while(1)
	{
		if(xTaskNotifyWait(0,0,(uint32_t*)&_adc_meas, portMAX_DELAY) == pdPASS)
		{
			//map the gathered value to the angle value
			_servo_position = _map(_adc_meas, 0, 4094, 0, 180);
			//set the position
			SERVO_set(&htim2, _servo_position);
		}
	}
}

void vI2CGather(void* pvParameters)
{

	while(1)
	{
		//FEED THE DEVICE WITH FIRST REGISTER ADDRESS
		HAL_I2C_Mem_Read_IT(&hi2c1, magnetic_module_addr, MAGNETIC_X_HIGH_REG, I2C_MEMADD_SIZE_8BIT, RAW_DATA, 6);

		//GET THE TEMPERATURE
		HAL_I2C_Mem_Read_IT(&hi2c1, magnetic_module_addr, MAGNETIC_TEMP_HIGH_REG, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&RAW_TEMP, 2);

	}

}

void vButton_IRQ(void)
{
	static uint8_t mode = 0;
	uint32_t timer = 0;
	switch(mode)
	{
	case 0:
		timer = 5;
		xTimerChangePeriodFromISR(Timer_1, pdMS_TO_TICKS(5), 0);
		mode++;
		break;
	case 1:
		timer = 10;
		xTimerChangePeriodFromISR(Timer_1, pdMS_TO_TICKS(10), 0);
		mode++;
		break;
	case 2:
		timer = 20;
		xTimerChangePeriodFromISR(Timer_1, pdMS_TO_TICKS(20), 0);
		mode++;
		break;
	case 3:
		timer = 100;
		xTimerChangePeriodFromISR(Timer_1, pdMS_TO_TICKS(100), 0);
		mode++;
		break;
	case 4:
		timer = 300;
		xTimerChangePeriodFromISR(Timer_1, pdMS_TO_TICKS(300), 0);
		mode++;
		break;
	case 5:
		timer = portMAX_DELAY;
		xTimerChangePeriodFromISR(Timer_1, portMAX_DELAY, 0);
		mode++;
		break;
	case 6:
		timer = 9999;
		MOTOR_set_position_ULN2003(99);
		xTimerChangePeriodFromISR(Timer_1, portMAX_DELAY, 0);
		mode = 0;
	default:
		timer = 1000;
		break;

	}
	xTaskNotifyFromISR(motor_stepper, timer, eSetValueWithOverwrite, 0);
}

void vTimer_1( TimerHandle_t xTimer )
{
	if(MOTOR_DIR)
	{
		MOTOR_left_ULN2003();
	}else
	{
		MOTOR_right_ULN2003();
	}
}
