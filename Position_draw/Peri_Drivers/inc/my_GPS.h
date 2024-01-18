/*
 * my_GPS.h
 *
 *  Created on: Nov 30, 2023
 *      Author: maciej
 */

#ifndef PERI_DRIVERS_INC_MY_GPS_H_
#define PERI_DRIVERS_INC_MY_GPS_H_

#include <stdint.h>
#include <string.h>

typedef struct {
	uint8_t hours;
	uint8_t minutes;
	uint8_t seconds;
}GPS_time_t;

typedef struct {
	GPS_time_t Time;
	uint8_t Latitude_deg;
	uint32_t Latitude_min;

	//minutes in degrees
	uint32_t Latitude_minINdegrees;

	uint8_t N_S_ind;
	uint8_t Longtitude_deg;
	uint32_t Longtitude_min;

	//minutes in degrees
	uint32_t Longtitude_minINdegrees;

	uint8_t E_W_ind;
}GPS_Position_Data_t;

GPS_Position_Data_t GPS_get_position(uint8_t * phrase, uint8_t size);

uint32_t data_get(uint8_t* wsk, int size, int data_ID, char ** ret_data_ppt);
uint32_t get_value_from_datastring(uint8_t* wsk, int data_ID);

#endif /* PERI_DRIVERS_INC_MY_GPS_H_ */


