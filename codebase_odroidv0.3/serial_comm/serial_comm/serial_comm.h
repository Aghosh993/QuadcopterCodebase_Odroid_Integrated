#ifndef SERIAL_COMM_H
#define SERIAL_COMM_H

#define _LITTLE_ENDIAN_

#include <stdint.h>

#include "AP_msg_types.h"

#ifdef __cplusplus
extern "C" {
#endif

  // data structure for accumulating some bytes (to send out to serial port)
typedef struct _CommBuffer
{
	unsigned char * s; // pointer to underlying memory buffer
	int n; // number of bytes currently used
	int max_n; // size of memory buffer
} CommBuffer;


typedef union
{
  unsigned char n[4];
  float f;
} float_chars;


  // RecvBuf is defined in AP_msg_types.h
  // initialize the RecvBuf and allocate "len" bytes -- *recvbuf should have been allocated before calling this function, but this function allocates recvbuf->buf
void init_recvbuf(RecvBuf *recvbuf , int len);

  // add a byte into the RecvBuf
void push_byte_recvbuf(RecvBuf *recvbuf, unsigned char c);

  // discard bytes until index k -- called when messages until this index have been processed and can now be discarded
void flush_till_pos(RecvBuf *recvbuf, int k);

  // find the index of the first received message in the RecvBuf
int find_first_msg(RecvBuf *recvbuf, MsgTypes *msgtype);


// initialize CommBuffer and allocate "size" bytes -- *cb should have been allocated before calling this function, but this function allocates cb->s
void init_commbuf(CommBuffer *cb, int size);

  // append n1 bytes from array pointed to by s into the CommBuffer
int add_to_commbuf(CommBuffer *cb, unsigned char *s , int n1); // returns non-zero if s cannot be appended into the existing contents in cb->s

  // re-initialize CommBuffer
void flush_commbuf(CommBuffer *cb);

#ifdef __cplusplus
}
#endif

#endif
