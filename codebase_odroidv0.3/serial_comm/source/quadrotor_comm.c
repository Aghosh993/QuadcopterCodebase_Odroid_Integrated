#include "quadrotor_comm.h"
#include "serial_comm/circular_buffer.h"
#include "serial_comm/serial_comm.h"
#include "serial_comm/AP_msg_callbacks.h"
#include "quadrotor_messages.h"
#include "quadrotor_message_handlers.h"

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>


// send_bytes assumed to be externally defined (or defined below)
void send_bytes(uint8_t *b, int n); // platform-dependent function to send an array of n bytes

#ifdef RUNNING_ON_AP
#include <hal_common_includes.h>
void send_bytes(uint8_t *b, int n)
{
  int i;
  for (i=0; i<n; i++)
  {
    usart_send_blocking(USART2, b[i]);
  }
}
#endif


// data structures for sending and receiving bytes
static Circular_Buffer *rx_circular_buffer;
static RecvBuf *rx_recvbuf;
static CommBuffer tx_send_buf;


// callback function to be called when an n-floats message is received
int received_nfloats_msg(NFloats_sMsg *msg)
{
  process_nfloats_msg(msg);
}

// callback function to be called when a string message is received
int received_str_msg(Str_sMsg *msg)
{
  process_str_msg(msg);
}

// initialize data structures used for sending and receiving bytes
void init_serial_comm_datastructures()
{
  rx_circular_buffer = (Circular_Buffer *) malloc(sizeof(Circular_Buffer));
  init_circular_buf(rx_circular_buffer);
  rx_recvbuf = (RecvBuf *) malloc(sizeof(RecvBuf));
  init_recvbuf(rx_recvbuf,600); // 600 bytes

  init_commbuf(&tx_send_buf,2500); // 2500 bytes
}

// send bytes out to the serial port and re-initialize the CommBuffer
void flush_txbuf_comm()
{
  send_bytes(tx_send_buf.s, tx_send_buf.n);
  flush_commbuf(&tx_send_buf);
}

// add to queue to send out
void add_bytes_to_txbuf(uint8_t *s, int n)
{
  int insufficient_space = add_to_commbuf(&tx_send_buf, s, n);
  if (insufficient_space)
  {
    flush_txbuf_comm();
    add_to_commbuf(&tx_send_buf, s, n);
  }
}

// examples of encoding and sending some messages
void test_send_some_messages()
{
	unsigned char s[128];
	{
		NFloats_sMsg msg;
		msg.n = 3;
		msg.f[0] = 1;
		msg.f[1] = 2;
		msg.f[2] = 3.3;
		int msglen = encode_nfloats_msg(&msg, s);
		add_bytes_to_txbuf(s, msglen);
	}

	{
		NFloats_sMsg msg;
		QR_State_Msg m;
		make_test_qr_state_msg(&m);
		encode_qr_state_msg(&m, &msg);
		int msglen = encode_nfloats_msg(&msg, s);
		add_bytes_to_txbuf(s, msglen);
	}
	{
		NFloats_sMsg msg;
		QR_Status_Msg m;
		make_test_qr_status_msg(&m);
		encode_qr_status_msg(&m, &msg);
		int msglen = encode_nfloats_msg(&msg, s);
		add_bytes_to_txbuf(s, msglen);
	}
	{
		NFloats_sMsg msg;
		QR_Motion_Command_Msg m;
		make_test_qr_motion_command_msg(&m);
		encode_qr_motion_command_msg(&m, &msg);
		int msglen = encode_nfloats_msg(&msg, s);
		add_bytes_to_txbuf(s, msglen);
	}
	{
		NFloats_sMsg msg;
		QR_Gimbal_Command_Msg m;
		make_test_qr_gimbal_command_msg(&m);
		encode_qr_gimbal_command_msg(&m, &msg);
		int msglen = encode_nfloats_msg(&msg, s);
		add_bytes_to_txbuf(s, msglen);
	}
	{
		NFloats_sMsg msg;
		QR_Land_Command_Msg m;
		make_test_qr_land_command_msg(&m);
		encode_qr_land_command_msg(&m, &msg);
		int msglen = encode_nfloats_msg(&msg, s);
		add_bytes_to_txbuf(s, msglen);
	}
	{
		char test_msg[] = "hello";
		Str_sMsg msg2;
		msg2.n = strlen(test_msg);
		strcpy(msg2.s, test_msg);
		int msglen = encode_str_msg(&msg2, s);
		add_bytes_to_txbuf(s, msglen);
	}


	flush_txbuf_comm();
}


// register callbacks for n-floats message, string message, etc.
void register_comm_callbacks()
{
  set_nfloats_msg_callback(received_nfloats_msg);
  set_str_msg_callback(received_str_msg);
  set_nfloats_id_msg_callback(received_qr_state_msg, QR_STATE_MSG_ID);
  set_nfloats_id_msg_callback(received_qr_status_msg, QR_STATUS_MSG_ID);
  set_nfloats_id_msg_callback(received_qr_motion_command_msg, QR_MOTION_COMMAND_ID);
  set_nfloats_id_msg_callback(received_qr_gimbal_command_msg, QR_GIMBAL_COMMAND_ID);
  set_nfloats_id_msg_callback(received_qr_land_command_msg, QR_LAND_COMMAND_ID);
}

// function to be called when some bytes are received (n: number of bytes, b: pointer to array of bytes)
void recv_bytes(uint8_t *b, int n)
{
  int i;
  for (i=0; i<n; i++) push_byte_circular_buf(rx_circular_buffer, b[i]);
}


// check for received messages
// copy from the circular buffer to the RecvBuf and then call update_callbacks_rxbuf_msgs
void check_for_messages()
{
  uint8_t _s[128];
  int n1 = get_bytes_circular_buf(rx_circular_buffer, _s,127);
  int i;
  for (i=0; i<n1; i++) push_byte_recvbuf(rx_recvbuf,_s[i]);
  update_callbacks_rxbuf_msgs(rx_recvbuf);
}

// send a string message -- sequence of n bytes
void send_str_msg(char *s, int n)
{
  Str_sMsg msg2;
  msg2.n = n;
  int i;
  for (i=0; i<n; i++)
  {
    msg2.s[i] = s[i];
  }
  //strcpy(msg2.s, test_msg);
  int msglen = encode_str_msg(&msg2, s);
  add_bytes_to_txbuf(s, msglen);
}

void send_land_command(QR_Land_Command_Msg *m)
{
	unsigned char s[128];
  NFloats_sMsg msg;
  encode_qr_land_command_msg(m, &msg);
  int msglen = encode_nfloats_msg(&msg, s);
  add_bytes_to_txbuf(s, msglen);
}

void send_gimbal_command(QR_Gimbal_Command_Msg *m)
{
	unsigned char s[128];
  NFloats_sMsg msg;
  encode_qr_gimbal_command_msg(m, &msg);
  int msglen = encode_nfloats_msg(&msg, s);
  add_bytes_to_txbuf(s, msglen);
}

void send_motion_command(QR_Motion_Command_Msg *m)
{
	unsigned char s[128];
  NFloats_sMsg msg;
  encode_qr_motion_command_msg(m, &msg);
  int msglen = encode_nfloats_msg(&msg, s);
  add_bytes_to_txbuf(s, msglen);
}
