/*
 * pxflow_reader.h
 *
 *  Created on: Jan 22, 2015
 *      Author: aghosh01
 */

#ifndef PXFLOW_READER_H_
#define PXFLOW_READER_H_

#include "mavlink.h"

typedef unsigned char uint8;

typedef struct {
	float x_velocity;
	float y_velocity;

	int16_t raw_x_flow;
	int16_t raw_y_flow;
  	
  	uint8_t quality;
  	float timestamp; // timestamp
} pxflow_flow_data_struct;

float get_pxflow_height(void);
void get_pxflow_flow_data(pxflow_flow_data_struct *buffer);

void px4flow_interrupt_callback(uint8 rxbyte);

int px4flow_received_new_data(); // returns 1 if new data received since last call to this function, returns 0 otherwise

#endif /* PXFLOW_READER_H_ */
