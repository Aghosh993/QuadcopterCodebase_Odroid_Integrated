/*
 * pxflow_reader.c
 *
 *  Created on: Jan 22, 2015
 *      Author: aghosh01
 */

#include "px4flow_reader.h"
#include "mission_timekeeper.h"

static pxflow_flow_data_struct last_received_flow_data;
static float last_received_height;
static float time_offset; // offset (in seconds) between pxflow timestamp and micrcontroller (timekeeper) timestamp
static int first_time = 1;
static int _received_new_data = 0;

// Something received - print out all bytes and parse packet
mavlink_message_t msg;
mavlink_status_t status;
mavlink_optical_flow_t optical_flow;

float get_pxflow_height(void)
{
	return last_received_height;
}
void get_pxflow_flow_data(pxflow_flow_data_struct *buffer)
{
	buffer->x_velocity = last_received_flow_data.x_velocity;
	buffer->y_velocity = last_received_flow_data.y_velocity;
	buffer->raw_x_flow = last_received_flow_data.raw_x_flow;
	buffer->raw_y_flow = last_received_flow_data.raw_y_flow;
	buffer->timestamp = last_received_flow_data.timestamp;
	buffer->quality = last_received_flow_data.quality;
}

void px4flow_interrupt_callback(uint8 rxbyte)
{
	if (mavlink_parse_char(MAVLINK_COMM_0, rxbyte, &msg, &status))
	{
		if(msg.msgid == MAVLINK_MSG_ID_OPTICAL_FLOW)
		{
			mavlink_msg_optical_flow_decode(&msg, &optical_flow);
      		
      		float t1 = optical_flow.time_usec * 0.001 * 0.001;
			
			if (first_time == 1)
			{
				time_offset = t1 - get_mission_time_as_millis()*0.001;
				first_time = 0;
			}
			
			last_received_flow_data.x_velocity = optical_flow.flow_comp_m_x;
			last_received_flow_data.y_velocity = optical_flow.flow_comp_m_y;

			last_received_flow_data.raw_x_flow = optical_flow.flow_x;
			last_received_flow_data.raw_y_flow = optical_flow.flow_y;
			
			last_received_flow_data.timestamp = t1 - time_offset ;

			last_received_height = optical_flow.ground_distance;

			last_received_flow_data.quality = optical_flow.quality;
			
			_received_new_data = 1;
		}
	}
}

int px4flow_received_new_data()
{
  int r = _received_new_data;
  _received_new_data = 0;
  return r;
}
