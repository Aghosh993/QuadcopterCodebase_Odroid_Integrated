#ifndef SIMPLE_TELEM_H
#define SIMPLE_TELEM_H	1

#include <hal_common_includes.h>

#define TELEMETRY_N_FLOATS_TO_SEND	12U
#define START_BYTE 		's'
#define CHKSUM_OFFSET	0x75

typedef enum {
	STATE_WAITING_FOR_START_BYTE,
	STATE_GETTING_TELEMETRY_DATA,
	STATE_GOT_PACKET
} telemetry_rx_state;

typedef struct {
        float px;
        float py;
        float vx;
        float vy;
        uint8_t not_found;
} odroid_packet_t;

void init_simple_telemetry_lib(void);
void send_n_floats(float *n_floats);
void telem_uart_rx_callback(uint8_t ch);
uint8_t new_odroid_message_available(void);
int get_latest_odroid_message(odroid_packet_t *message);

#endif