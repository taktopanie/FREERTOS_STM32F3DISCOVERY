/*
 * task_handler.c
 *
 *  Created on: Oct 13, 2023
 *      Author: taktopanie
 */


#include "main.h"
#include <string.h>

void LED_task_handler(void* parameters){
	uint32_t cmd_addr;
	command_t *cmd;
	const char* msg_led = 	"========================\n"
							"|      LED effects      \n"
							"========================\n"
							"(NONE,e1,e2,e3,e4)\n"
							"|         MENU         |\n"
							"Enter your choice here : ";

	const char* msg_invalid="========================\n"
						  	"|    invalid option    |\n"
							"========================\n";
	while(1){

		//wait for notify
		xTaskNotifyWait(0,0,NULL,portMAX_DELAY);

		//print led menu
		xQueueSend(q_print,&msg_led, portMAX_DELAY);

		//wait for LED command (notify wait)
		xTaskNotifyWait(0,0,&cmd_addr,portMAX_DELAY);

		cmd = (command_t*)cmd_addr;

		if(cmd->len <= 4){
			if(!strcmp((char*)cmd->payload,"none")){
				led_effect_stop();
			}else if(!strcmp((char*)cmd->payload,"e1")){
				led_effect(1);
			}else if(!strcmp((char*)cmd->payload,"e2")){
				led_effect(2);
			}else if(!strcmp((char*)cmd->payload,"e3")){
				led_effect(3);
			}else if(!strcmp((char*)cmd->payload,"e4")){
				led_effect(4);
			}else{
				xQueueSend(q_print, &msg_invalid, portMAX_DELAY);
			}
		}else{
			xQueueSend(q_print, &msg_invalid, portMAX_DELAY);
		}
		curr_state = sMainMenu;

		xTaskNotify(MENU_handle, 0, eNoAction);
	}
}

void MENU_task_handler (void* parameters){
	uint32_t cmd_addr;
	command_t *cmd;

	int option;

	const char* msg_menu =	"========================\n"
						  	"|         MENU         |\n"
							"========================\n"
							"LED effect     ----> 0  \n"
							"Date and time  ----> 1  \n"
							"Exit           ----> 2  \n"
							"Enter your choice here : ";

	const char* msg_invalid="========================\n"
						  	"|    invalid option    |\n"
							"========================\n";



	while(1){
		xQueueSend(q_print, &msg_menu, portMAX_DELAY);

		xTaskNotifyWait(0,0, &cmd_addr, portMAX_DELAY);

		cmd = (command_t*)cmd_addr;

		if(cmd->len == 1){
			//converting ascii to number
			option = cmd->payload[0]-48;

			switch(option){
			case 0:
				curr_state = sLedEffect;
				xTaskNotify(LED_handle, 0, eNoAction);
			break;

			case 1:
				curr_state = sRtcMenu;
				xTaskNotify(RTC_handle, 0, eNoAction);
			break;

			case 2:
				//implement exit;
			break;

			default:
				xQueueSend(q_print, &msg_invalid, portMAX_DELAY);
			}

		}else{
			// invalid entry
			xQueueSend(q_print, &msg_invalid, portMAX_DELAY);
			continue;
		}

		//wait to run again when some other task notifies
		xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);

	}

}

void RTC_task_handler (void* parameters){

	while(1){

	}

}

void PRINT_task_handler (void* parameters){

	while(1){

	}

}

void COMMAND_task_handler (void* parameters){

	BaseType_t ret;
	command_t cmd;

	while(1){
		//Implement notify wait
		ret = xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);

		// if received notification
		if(ret == pdTRUE){
			process_command(&cmd);

		}
		//process the user data(command) stored in input data queue

		//notify the command to relevant task

	}

}

void process_command(command_t *cmd){
	extract_command(cmd);

	switch(curr_state){
	case sMainMenu:
		xTaskNotify(MENU_handle, (uint32_t)cmd, eSetValueWithOverwrite);
		break;

	case sLedEffect:
		xTaskNotify(LED_handle, (uint32_t)cmd, eSetValueWithOverwrite);
		break;

	case sRtcMenu:
	case sRtcTimeConfig:
	case sRtcDateConfig:
	case sRtcReport:
		xTaskNotify(COMMAND_handle, (uint32_t)cmd, eSetValueWithOverwrite);
		break;

	}

}

int extract_command(command_t *cmd){
	uint8_t item;
	BaseType_t status;

	status = uxQueueMessagesWaiting(q_data);
	if(!status){
		return -1;
	}

	uint8_t i = 0;

	do{
		status = xQueueReceive(q_data, &item, 0);
		if(status == pdTRUE){
			cmd->payload[i++] = item;
		}
	}while(item != '\n');

	cmd->payload[i-1] = '\0';
	cmd->len = i-1;
	return 0;
}
