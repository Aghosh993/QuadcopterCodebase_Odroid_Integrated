#ifndef SERIAL_USB_H
#define SERIAL_USB_H

#include <stdint.h>

class Serial_USB
{
public:
  Serial_USB();
  void send_bytes(uint8_t *b, int n); // send an array of n bytes
  int read_some_bytes(uint8_t *b, int n); // read a max of n bytes into the array -- returns number of bytes read
};


extern "C" {
void init_serial_usb_comm();
void update_serial_usb_comm();
void send_bytes(uint8_t *b, int n);
}

#endif
