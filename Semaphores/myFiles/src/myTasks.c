/*
 * myTasks.c
 *
 *  Created on: Nov 14, 2023
 *      Author: maciej
 */

#include "main.h"

uint32_t notify_task;

char * wsk;

void __UART_print(char * msg){
			while ( __HAL_UART_GET_FLAG(&huart3, UART_FLAG_TXE) != SET);
			HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
			while ( __HAL_UART_GET_FLAG(&huart3, UART_FLAG_TC) != SET);
}


void UART_task (void *pvParameters){

	//uint32_t notify_task;

	char msg [100];
	//const char * text = "hello from queue\n";

	char msg_2 [20] = "Hello from queue\n";

	while(1){
		xTaskNotifyWait(0, 0, &notify_task, portMAX_DELAY);
		sprintf(msg, "UART task running %d, time...\n\r", (int)notify_task);
		__UART_print(msg);

		for(int i = 0; i < 20; i++){
			xQueueSend(xWorkQueue, &msg_2[i], portMAX_DELAY);
		}

		sprintf(msg,"Sent: %d cmd's", (int)notify_task);

		lcd_send_command(SET_DDRAM_ADDR | LCD_LINE2);
		lcd_send_text(msg);
	}
}


void LCD_task (void *pvParameters){

	uint8_t BUFFOR = 20;

	char table [BUFFOR];


	int i = 0;

	  lcd_init();
	  //set cursor to 0x0
	  lcd_send_command(LCD_CLEAR);
	  lcd_send_text("Working...");

	while(1){
		while(table[i-1] != '\n'){
			xQueueReceive(xWorkQueue,&table[i++], portMAX_DELAY);

			//table overflow
			configASSERT(i < BUFFOR);

		}

		// fill rest of the table
		for(int k = sizeof(table)/sizeof(char); i < k; i++){
			table[i] = '\0';
		}

		xQueueReset(xWorkQueue);
		__UART_print(table);

		i=0;

	}
}

