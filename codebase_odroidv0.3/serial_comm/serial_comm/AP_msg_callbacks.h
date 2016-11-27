#ifndef AP_MSGS_CALLBACKS_H
#define AP_MSGS_CALLBACKS_H

#include "AP_msg_types.h"
#include "serial_comm.h"

#ifdef __cplusplus
extern "C" {
#endif

  // callback function prototypes -- n-floats message, n-floats message with ID, string message
  typedef int (*NFloatsMsgCallbackFuncType) (NFloats_sMsg *msg);
  typedef int (*NFloatsIdMsgCallbackFuncType) (NFloats_sMsg *msg);
  typedef int (*StrMsgCallbackFuncType) (Str_sMsg *msg);

  // register callback functions
  void set_nfloats_msg_callback(NFloatsMsgCallbackFuncType f);
  int set_nfloats_id_msg_callback(NFloatsIdMsgCallbackFuncType f, int id); // returns 0 if successfully set, < 0 if not
  void set_str_msg_callback(StrMsgCallbackFuncType f);

  // check for messages in the RecvBuf and call any relevant callback functions
  void update_callbacks_rxbuf_msgs(RecvBuf *b);



#ifdef __cplusplus
}
#endif


#endif
