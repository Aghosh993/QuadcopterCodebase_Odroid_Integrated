#ifndef AP_MSG_TYPES_H
#define AP_MSG_TYPES_H


#ifdef __cplusplus
extern "C" {
#endif

  // two fundamental types of messages -- n-floats and string -- ID-based message format is layered on top of n-floats messages
typedef enum{
    INVALID_MSG = -1,
    NFLOATS_MSG = 1,
    STR_MSG = 2
} MsgTypes;

  // max number of floats in n-floats message
#define MAX_N_FLOATS_MSG 32

  // max length of a string in a string message
#define MAX_STRLEN_STR_MSG 1000

  // data structure for defining n-floats message -- n is the number of floats in the message
typedef struct _NFloats_sMsg
{
    float f[MAX_N_FLOATS_MSG];
    int n;
} NFloats_sMsg;

  // data structure for defining string message -- n is the length of the string in the message
typedef struct _Str_sMsg
{
    unsigned char s[MAX_STRLEN_STR_MSG];
    int n;
} Str_sMsg;

  // buffer into which bytes are accumulated
typedef struct _RecvBuf
{
    unsigned char *buf;
    int len;
    int valid_pos;
} RecvBuf;


  // encode n-floats message as a char array
int encode_nfloats_msg(NFloats_sMsg *msg, unsigned char *s);

  // extract n-floats message from a char array starting from position pos in recvbuf -- msglen is the length of the message
int extract_nfloats_msg(RecvBuf *recvbuf, int pos, NFloats_sMsg *msg, int *msglen);

  // encode string message as a char array
int encode_str_msg(Str_sMsg *msg, unsigned char *s);

  // extract string message from a char array starting from position pos in recvbuf -- msglen is the length of the message
int extract_str_msg(RecvBuf *recvbuf, int pos, Str_sMsg *msg, int *msglen);


#ifdef __cplusplus
}
#endif

#endif
