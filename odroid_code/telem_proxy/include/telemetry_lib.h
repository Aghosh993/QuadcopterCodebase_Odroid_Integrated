#ifndef SIMPLE_TELEM_H
#define SIMPLE_TELEM_H	1

#include <serialport_linux.h>

#define TELEMETRY_N_FLOATS_TO_RECV	12U
#define CHKSUM_OFFSET	0x75

void st_telemetry_channel_init(char* ser_port);

int recv_n_floats(float *n_floats, uint8_t *input, uint8_t received_checksum);

int verify_checksum(uint8_t *message, uint8_t received_checksum);

#endif