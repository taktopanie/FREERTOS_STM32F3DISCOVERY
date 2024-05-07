/*
 * myTasks.h
 *
 *  Created on: May 7, 2024
 *      Author: maciej
 */

#ifndef INC_MYTASKS_H_
#define INC_MYTASKS_H_

#include "main.h"

extern TaskHandle_t state_update_hndl;
extern TaskHandle_t LCD_hndl;

extern TimerHandle_t setup_timer_hndl;

extern uint8_t push_state;

void state_update_task(void* vParameters);
void LCD_task(void* vParameters);
void setup_timer_expiry(TimerHandle_t xTimer);

#endif /* INC_MYTASKS_H_ */
