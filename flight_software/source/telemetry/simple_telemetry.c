#include <simple_telemetry.h>

static telemetry_rx_state rx_state;
static uint8_t data_buffer_rx_iterator;
static uint8_t rx_data_buffer[sizeof(odroid_packet_t) + 1U]; // +1 for checksum

void init_simple_telemetry_lib(void)
{
	rx_state = STATE_WAITING_FOR_START_BYTE;
	data_buffer_rx_iterator = 0U;

	int i = 0;
	for(i = 0; i < sizeof(odroid_packet_t) + 1U; ++i)
	{
		rx_data_buffer[i] = 0U;
	}	
}

void send_n_floats(float *n_floats)
{
	uint8_t checksum = 0U;

	union {
		float data_to_encode[TELEMETRY_N_FLOATS_TO_SEND];
		uint8_t output_buffer[TELEMETRY_N_FLOATS_TO_SEND * 4U];
	} n_floats_to_telemetry_stream;

	uint8_t i = 0U;

	for(i = 0U; i < TELEMETRY_N_FLOATS_TO_SEND; ++i)
	{
		n_floats_to_telemetry_stream.data_to_encode[i] = n_floats[i];
	}

	usart_send_blocking(USART2, START_BYTE);

	for(i = 0U; i < TELEMETRY_N_FLOATS_TO_SEND * 4U; ++i)
	{
		usart_send_blocking(USART2, n_floats_to_telemetry_stream.output_buffer[i]);
		checksum += n_floats_to_telemetry_stream.output_buffer[i];
	}

	usart_send_blocking(USART2, (checksum + CHKSUM_OFFSET) & 0xff);
}

void telem_uart_rx_callback(uint8_t ch)
{
	switch(rx_state)
	{
		case STATE_WAITING_FOR_START_BYTE:
			if(ch == START_BYTE){
				rx_state = STATE_GETTING_TELEMETRY_DATA;
			}
			break;
		case STATE_GETTING_TELEMETRY_DATA:
			if(data_buffer_rx_iterator < sizeof(odroid_packet_t)+1U)
			{
				rx_data_buffer[data_buffer_rx_iterator] = ch;
				++data_buffer_rx_iterator;

				if(data_buffer_rx_iterator == sizeof(odroid_packet_t) + 1U)
				{
					rx_state = STATE_GOT_PACKET;
					data_buffer_rx_iterator = 0U;
				}
			}
			break;
		case STATE_GOT_PACKET:
			break;
	}
}

uint8_t new_odroid_message_available(void)
{
	if(rx_state == STATE_GOT_PACKET)
	{
		return 1U;
	}
	return 0U;
}

int get_latest_odroid_message(odroid_packet_t *message)
{
	union {
	        odroid_packet_t output_data;
	        uint8_t input_data[sizeof(odroid_packet_t)];
	} data_buffer_to_odroid_packet;

	uint8_t received_checksum = 0U;
	uint16_t calculated_checksum = 0U;

	int i = 0;

	for(i = 0; i < sizeof(odroid_packet_t); ++i)
	{
		data_buffer_to_odroid_packet.input_data[i] = rx_data_buffer[i];
		calculated_checksum += rx_data_buffer[i];
	}
	received_checksum = rx_data_buffer[sizeof(odroid_packet_t)];
	calculated_checksum += CHKSUM_OFFSET;

	if(received_checksum != (uint8_t)(calculated_checksum & 0xff))
	{
		message->px = 0.0f;
		message->py = 0.0f;
		message->vx = 0.0f;
		message->vy = 0.0f;
		message->not_found = 0U;
		rx_state = STATE_WAITING_FOR_START_BYTE;
		return -1;
	}
	else
	{
		message->px = data_buffer_to_odroid_packet.output_data.px;
		message->py = data_buffer_to_odroid_packet.output_data.py;
		message->vx = data_buffer_to_odroid_packet.output_data.vx;
		message->vy = data_buffer_to_odroid_packet.output_data.vy;
		message->not_found = data_buffer_to_odroid_packet.output_data.not_found;
		rx_state = STATE_WAITING_FOR_START_BYTE;
		return 0;
	}
}