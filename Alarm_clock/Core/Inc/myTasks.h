/*
 * myTasks.h
 *
 *  Created on: May 7, 2024
 *      Author: maciej
 */

#ifndef INC_MYTASKS_H_
#define INC_MYTASKS_H_

#include "main.h"
#include "stdlib.h"

//TASKS
extern TaskHandle_t STATE_UPDATE_HNDL;
extern TaskHandle_t LCD_HNDL;
extern TaskHandle_t DS3231_HNDL;

extern TaskHandle_t DOUBLE_CLICK_HNDL;
extern TaskHandle_t SHORT_CLICK_HNDL;
extern TaskHandle_t LONG_PRESS_HNDL;

extern TaskHandle_t CLOCK_TICK_HNDL;

//TIMERS
extern TimerHandle_t setup_timer_hndl;
extern TimerHandle_t BUTTON_TIMER;

extern uint8_t push_state;

extern I2C_HandleTypeDef hi2c1;


void state_update_task(void* vParameters);
void LCD_task(void* vParameters);
void DS3231_task(void* vParameters);
void setup_timer_expiry(TimerHandle_t xTimer);

void SHORT_CLICK_task(void* vParameters);
void DOUBLE_CLICK_task(void* vParameters);
void LONG_PRESS_task(void* vParameters);

void CLOCK_TICK_task(void* vParameters);

//private functions
void _CLOCK_second_increment(void);



#endif /* INC_MYTASKS_H_ */
