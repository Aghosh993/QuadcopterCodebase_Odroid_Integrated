#include "telemetry_lib.h"

void st_telemetry_channel_init(char* ser_port)
{
	setup_linux_serial(ser_port);
}

int recv_n_floats(float *n_floats, uint8_t *input, uint8_t received_checksum)
{
	uint8_t checksum = 0U;

	union {
		uint8_t input_buffer[TELEMETRY_N_FLOATS_TO_RECV * 4U];
		float decoded_data[TELEMETRY_N_FLOATS_TO_RECV];
	} telemetry_stream_to_n_floats;

	uint8_t i = 0U;

	for(i = 0U; i < TELEMETRY_N_FLOATS_TO_RECV * 4U; ++i)
	{
		telemetry_stream_to_n_floats.input_buffer[i] = input[i];
	}

	for(i = 0U; i < TELEMETRY_N_FLOATS_TO_RECV; ++i)
	{
		n_floats[i] = telemetry_stream_to_n_floats.decoded_data[i];
	}
	return verify_checksum(input, received_checksum);
}

int verify_checksum(uint8_t *message, uint8_t received_checksum)
{
	uint16_t calc_checksum = 0U;

	uint8_t i = 0U;

	for(i = 0U; i < TELEMETRY_N_FLOATS_TO_RECV * 4U; ++i)
	{
		calc_checksum += message[i];
	}

	calc_checksum += CHKSUM_OFFSET;

	if((uint8_t)(calc_checksum & 0xff) != received_checksum)
	{
		return -1; // Failure
	}
	return 0; // Success
}