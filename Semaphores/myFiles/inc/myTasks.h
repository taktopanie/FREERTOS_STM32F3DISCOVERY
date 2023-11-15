/*
 * myTasks.h
 *
 *  Created on: Nov 14, 2023
 *      Author: maciej
 */

#ifndef INC_MYTASKS_H_
#define INC_MYTASKS_H_

#include "main.h"


void UART_task (void *pvParameters);
void LCD_task (void *pvParameters);

#endif /* INC_MYTASKS_H_ */
