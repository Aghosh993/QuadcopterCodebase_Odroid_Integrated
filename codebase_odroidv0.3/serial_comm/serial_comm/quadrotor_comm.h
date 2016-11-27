#ifndef SERIAL_COMM_EXAMPLES_H
#define SERIAL_COMM_EXAMPLES_H

#include "serial_comm_platform_specific.h"

#include <stdint.h>
#include "quadrotor_messages.h"

#ifdef __cplusplus
extern "C" {
#endif

  // initialize data structures used for sending and receiving bytes
void init_serial_comm_datastructures();

  // examples of encoding and sending some messages
void test_send_some_messages();

  // function to be called when some bytes are received (n: number of bytes, b: pointer to array of bytes)
void recv_bytes(uint8_t *b, int n);

  // check for received messages
void check_for_messages();

  // register callbacks for n-floats message, string message, etc.
  void register_comm_callbacks();
  
  // send a string message -- sequence of n bytes
  void send_str_msg(char *s, int n);

  void send_land_command(QR_Land_Command_Msg *m);
  void send_gimbal_command(QR_Gimbal_Command_Msg *m);
  void send_motion_command(QR_Motion_Command_Msg *m);
  
  void flush_txbuf_comm();



#ifdef __cplusplus
}
#endif

#endif
