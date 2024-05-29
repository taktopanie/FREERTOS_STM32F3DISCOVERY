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

extern TaskHandle_t DOUBLE_CLICK_HNDL;
extern TaskHandle_t SHORT_CLICK_HNDL;
extern TaskHandle_t LONG_PRESS_HNDL;

//TIMERS
extern TimerHandle_t setup_timer_hndl;
extern TimerHandle_t BUTTON_TIMER;

extern uint8_t push_state;

void state_update_task(void* vParameters);
void LCD_task(void* vParameters);
void setup_timer_expiry(TimerHandle_t xTimer);

void SHORT_CLICK_task(void* vParameters);
void DOUBLE_CLICK_task(void* vParameters);
void LONG_PRESS_task(void* vParameters);




#endif /* INC_MYTASKS_H_ */
