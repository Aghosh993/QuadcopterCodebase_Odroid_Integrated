#ifndef SERIALPORT_HAL_H
#define SERIALPORT_HAL_H 1

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <stdint.h>

void setup_linux_serial(char* serialPort_name);
void uart_send_byte_linux(uint8_t xmit_byte);
uint8_t recv_byte();
void recv_data(uint8_t *data_buf, uint8_t len);

int get_serial_fd(void);

#endif