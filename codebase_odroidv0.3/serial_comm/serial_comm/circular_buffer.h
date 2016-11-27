#ifndef CIRCULAR_BUFFER_H
#define CIRCULAR_BUFFER_H

// circular buffer data structure

#ifdef __cplusplus
extern "C" {
#endif

  // default length of a Circular_Buffer
#define CIRCULARBUFLEN 256
typedef struct _Circular_Buffer
{
  unsigned char *buf; // pointer to underlying memory buffer
  int n_w; // write index
  int n_r; // read index
  int len; // length of the buffer (in bytes)
} Circular_Buffer;

  // initialize the circular buffer -- *cb should have been allocated before calling this function, but this function allocates cb->buf 
void init_circular_buf(Circular_Buffer *cb);

  // change the size of the circular buffer -- de-allocates cb->buf and allocates again to requested size
void resize_circular_buf(Circular_Buffer *cb, int n);

  // push a byte into the circular buffer
void push_byte_circular_buf(Circular_Buffer *cb , unsigned char b);

  // copy up to b1_len bytes from the circular buffer into the array pointed to by b1_len -- returns the number of bytes actually copied 
int get_bytes_circular_buf(Circular_Buffer *cb, unsigned char *b1, int b1_len);

#ifdef __cplusplus
}
#endif


#endif
