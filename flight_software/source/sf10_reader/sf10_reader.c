/*
 * sf10_reader.c
 *
 *  Created on: Jul 2, 2015
 *      Author: aghosh01
 */

#include "sf10_reader.h"
#include "mission_timekeeper.h"

static int _received_new_data = 0;

sf10_sensor_data_handler *create_new_sf10_data_handler(uint8_t timeout_limit, float max_height_possible, void* uart_tx_func_pointer)
{
	sf10_sensor_data_handler *handler_ret =
			(sf10_sensor_data_handler *)malloc(sizeof(sf10_sensor_data_handler));
	handler_ret->ncycles_timeout_limit = timeout_limit;
	handler_ret->last_received_height = 0.0f;
	handler_ret->max_height_possible = max_height_possible;
	handler_ret->message_buffer_iterator = 0U;
	handler_ret->height_variable_lock = RESOURCE_FREE;
	handler_ret->uart_tx_function_ptr = uart_tx_func_pointer;
	return handler_ret;
}

void sf10_reader_callback(sf10_sensor_data_handler *dh, uint8_t c)
{
	uint8_t decimal_place_position = 0U;
	switch(dh->acq_st)
	{
	case STATE_IDLE:
		dh->acq_st = STATE_ERROR;
		break;
	case STATE_SENT_DATA_REQUEST_BYTE:
		dh->acq_st = STATE_GETTING_DATA;
		dh->message_buffer_iterator = 0U;
		dh->received_data_raw[dh->message_buffer_iterator] = c;
		++(dh->message_buffer_iterator);
		break;
	case STATE_GETTING_DATA:
		/*
		 * Continue filling up the incoming data buffer and increment pointer:
		 */
		if(dh->message_buffer_iterator < MESSAGE_BUFFER_MAX_LEN)
		{
			dh->received_data_raw[dh->message_buffer_iterator] = c;
			++(dh->message_buffer_iterator);
		}
		/*
		 * If we reach the end of a data output from the sensor,
		 * handle as appropriate:
		 */
		if(c == '\r' || c == '\n')
		{
			dh->last_received_height = 0.0f;
			decimal_place_position = find_decimal_place(dh->received_data_raw);
			if(decimal_place_position >= MESSAGE_BUFFER_MAX_LEN)
			{
				dh->acq_st = STATE_ERROR;
			}
			else
			{
				uint8_t iter = 0U;
				if(dh->height_variable_lock == RESOURCE_FREE)
				{
					dh->height_variable_lock = RESOURCE_ALLOC_TO_ISR;
					for(iter=0U;iter<dh->message_buffer_iterator;++iter)
					{
						if(dh->received_data_raw[iter] == '\r' || dh->received_data_raw[iter] == '\n')
						{
							break;
						}
						else 
						{
							// printf("%d\n", dh->received_data_raw[iter] - '0');
							if(iter < decimal_place_position)
							{
								dh->last_received_height += (float)(dh->received_data_raw[iter] - '0')*powers_of_ten(decimal_place_position-iter-1);
							}
							if(iter > decimal_place_position)
							{
								dh->last_received_height += (float)(dh->received_data_raw[iter] - '0')*powers_of_ten(decimal_place_position-iter);
							}
						}
					}
          _received_new_data = 1;
          dh->timestamp = get_mission_time_as_millis()*0.001;
				}
				dh->height_variable_lock = RESOURCE_FREE;
			}
			dh->acq_st = STATE_GOT_NEW_MEASUREMENT;
			dh->message_buffer_iterator = 0U;
		}
		break;
	case STATE_GOT_NEW_MEASUREMENT:
		dh->acq_st = STATE_ERROR;
		break;
	case STATE_ERROR:
		dh->acq_st = STATE_IDLE;
		break;
	}
}

void request_sf10_sensor_update(sf10_sensor_data_handler *dh)
{
	dh->uart_tx_function_ptr(SF10_DATA_REQUEST_BYTE);
	dh->acq_st = STATE_SENT_DATA_REQUEST_BYTE;
}

float get_last_sf10_sensor_height(sf10_sensor_data_handler *dh)
{
	float local_copy = 0.0f;
	dh->height_variable_lock = RESOURCE_ALLOC_TO_USER_PROG;
	local_copy = dh->last_received_height;
	dh->height_variable_lock = RESOURCE_FREE;
	return local_copy;
}

float get_last_sf10_timestamp(sf10_sensor_data_handler *dh)
{
	float local_copy = 0.0f;
	dh->height_variable_lock = RESOURCE_ALLOC_TO_USER_PROG;
	local_copy = dh->timestamp;
	dh->height_variable_lock = RESOURCE_FREE;
	return local_copy;
}

static uint8_t find_decimal_place(uint8_t buffer[MESSAGE_BUFFER_MAX_LEN])
{
	uint8_t i = 0U;
	for(i = 0U; i < MESSAGE_BUFFER_MAX_LEN; ++i)
	{
		if(buffer[i] == '.')
		{
			return i;
		}
	}
	/*
	 * Return max value of an unsigned 8-bit integer to indicate error,
	 * since message buffer is not going to be that high in length anyway:
	 */
	return 255U;
}

static float powers_of_ten(int place)
{
	switch(place)
	{
	case -3:
		return 0.001f;
	case -2:
		return 0.010f;
	case -1:
		return 0.100f;
	case 0:
		return 1.000f;
	case 1:
		return 10.00f;
	case 2:
		return 100.0f;
	default:
		return 0.000f;
	}
}

int sf10_received_new_data()
{
  int r = _received_new_data;
  _received_new_data = 0;
  return r;
}
