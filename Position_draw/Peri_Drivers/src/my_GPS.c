/*
 * my_GPS.c
 *
 *  Created on: Nov 30, 2023
 *      Author: maciej
 */

#include <my_GPS.h>

int char2int(char zn)
{
	return (zn - 48);
}

GPS_Position_Data_t GPS_get_position(uint8_t * phrase, uint8_t size)
{
	GPS_Position_Data_t _tmp = {{0,0,0},0,0,0,0,0,0};

	if(!(strncmp((char*)phrase, "GPGGA", 5)))
	{
		char * tmp_ppt = 0;
		uint32_t data_length = 0;

		data_length = data_get(phrase, size, 0 , &tmp_ppt);
		if(data_length)
		{
			_tmp.Time.hours = (char2int(*tmp_ppt))*10 + char2int(*(tmp_ppt+1));
			_tmp.Time.minutes = char2int(*(tmp_ppt+2))*10 + char2int(*(tmp_ppt+3));
			_tmp.Time.seconds = char2int(*(tmp_ppt+4))*10 + char2int(*(tmp_ppt+5));
		}


		data_length = data_get(phrase, size, 1 , &tmp_ppt);
		if(data_length)
		{
			_tmp.Latitude_deg = (char2int(*tmp_ppt))*10 + (char2int(*(tmp_ppt+1)));
			_tmp.Latitude_min = (char2int( *(tmp_ppt+2) ) )*100000 + \
					(char2int( *(tmp_ppt+3) ))*10000 + \
					//comma sign
					(char2int( *(tmp_ppt+5) ))*1000 + \
					(char2int( *(tmp_ppt+6) ))*100 + \
					(char2int( *(tmp_ppt+7) ))*10 + \
					(char2int( *(tmp_ppt+8) ));
		}

		_tmp.Latitude_minINdegrees = (uint32_t)_tmp.Latitude_min * 166.66667;

		data_length = data_get(phrase, size, 2 , &tmp_ppt);
		if(data_length)
		{
			_tmp.N_S_ind = *tmp_ppt;
		}


		data_length = data_get(phrase, size, 3 , &tmp_ppt);
		if(data_length)
		{
			_tmp.Longtitude_deg = (char2int( *tmp_ppt) )*100 + \
					(char2int( *(tmp_ppt+1) )*10) + \
					(char2int( *(tmp_ppt+2) ));
			_tmp.Longtitude_min = (char2int( *(tmp_ppt+3) ) )*100000 + \
					(char2int( *(tmp_ppt+4) ))*10000 + \
					//comma sign
					(char2int( *(tmp_ppt+6) ))*1000 + \
					(char2int( *(tmp_ppt+7) ))*100 + \
					(char2int( *(tmp_ppt+8) ))*10 + \
					(char2int( *(tmp_ppt+9) ));
		}

		_tmp.Longtitude_minINdegrees = (uint32_t)(_tmp.Longtitude_min * 166.666667);

		data_length = data_get(phrase, size, 4 , &tmp_ppt);
		if(data_length)
		{
			_tmp.E_W_ind = *tmp_ppt;
		}



	}
//	else{
//		_tmp.Latitude = 0;
//		_tmp.Longtitude = 0;
//		_tmp.E_W_ind = 0;
//		_tmp.N_S_ind = 0;
//	}
	return _tmp;

}
/*
 * args:
 * wsk - pointer to phrase from GPS ex. GPGGA...
 * size - sizeof phrase under wsk pointer
 * data_ID - data index which has to be decoded			SEE BELOW
 * ret_data_ppt - char pointer to first letter of the result
 *
 * ret:
 * - number of data under ret_data_ppt which may be decoded
 *
 ****************************************************************
 *
 * - data_ID example: $GPGGA,aaa,bbb,ccc,ddd,eee...
 * 		aaa field ID: 0
 * 		bbb field ID: 1
 * 		ccc field ID: 2
 * 		...
 *
 ****************************************************************
 *
 */

uint32_t data_get(uint8_t* wsk, int size, int data_ID, char ** ret_data_ppt)
{

	//make copy of data
	char table [size];
	for(int i =0; i < size; i++)
	{
		table[i] = *(wsk + i);
	}

	char * tmp = NULL;

	// DELETE prev ',' signs
	for (int i = 0; i < data_ID; i ++){
		tmp = strchr(table, ',');
		*tmp = '/';
	}
	tmp = NULL;

	//GET HEAD and TAIL of msg with data_ID which will be returned
	char * head;
	char * tail;

	head = strchr(table, ',');
	*head = '/';
	tail = strchr(table, ',');

	//No. of data available and pointer to the data
	*ret_data_ppt = (char*)(wsk + (head - table) + 1);
	return ((tail-head) - 1);

}

uint32_t get_value_from_datastring(uint8_t* wsk, int data_ID){
 return 0;
}
