/*
 * sf10_reader.h
 *
 *  Created on: Jul 2, 2015
 *      Author: aghosh01
 */

#ifndef SF10_READER_H_
#define SF10_READER_H_	1

#include <stdint.h>
#include <stdlib.h>

#include "sf10_hal_stm32.h"

#define MESSAGE_BUFFER_MAX_LEN 	6
#define MAX_HEIGHT_SF10_A		25.00f

#define SF10_DATA_REQUEST_BYTE	'R'

typedef enum
{
	STATE_IDLE,
	STATE_SENT_DATA_REQUEST_BYTE,
	STATE_GETTING_DATA,
	STATE_GOT_NEW_MEASUREMENT,
	STATE_ERROR
} sf10_data_acquisition_state;

typedef enum
{
	RESOURCE_ALLOC_TO_ISR,
	RESOURCE_ALLOC_TO_USER_PROG,
	RESOURCE_FREE
} resource_allocator_enum;

typedef struct
{
	float last_received_height;
  float timestamp;
	uint8_t ncycles_timeout_limit;
	sf10_data_acquisition_state acq_st;

	uint8_t received_data_raw[MESSAGE_BUFFER_MAX_LEN];
	uint8_t message_buffer_iterator;

	float max_height_possible;

	resource_allocator_enum height_variable_lock;
	void (*uart_tx_function_ptr)(uint8_t);
} sf10_sensor_data_handler;


sf10_sensor_data_handler *create_new_sf10_data_handler(uint8_t timeout_limit, float max_height_possible, void* uart_tx_func_pointer);
void sf10_reader_callback(sf10_sensor_data_handler *dh, uint8_t c);
void request_sf10_sensor_update(sf10_sensor_data_handler *dh);
float get_last_sf10_sensor_height(sf10_sensor_data_handler *dh);
float get_last_sf10_timestamp(sf10_sensor_data_handler *dh);

int sf10_received_new_data(); // returns 1 if new data received since last call to this function, returns 0 otherwise

/*
 * (Internal) Helper functions:
 */
static uint8_t find_decimal_place(uint8_t buffer[MESSAGE_BUFFER_MAX_LEN]);
static float powers_of_ten(int place);

#endif /* SF10_READER_H_ */
