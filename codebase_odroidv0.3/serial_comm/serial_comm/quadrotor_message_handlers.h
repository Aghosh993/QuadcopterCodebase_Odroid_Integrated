#ifndef QUADROTOR_MESSAGE_HANDLERS_H
#define QUADROTOR_MESSAGE_HANDLERS_H

#include "quadrotor_messages.h"

#ifdef __cplusplus
extern "C" {
#endif

void process_qr_state_msg(QR_State_Msg *m);
void process_qr_status_msg(QR_Status_Msg *m);
void process_qr_motion_command_msg(QR_Motion_Command_Msg *m);
void process_qr_gimbal_command_msg(QR_Gimbal_Command_Msg *m);
void process_qr_land_command_msg(QR_Land_Command_Msg *m);
void process_nfloats_msg(NFloats_sMsg *msg);
void process_str_msg(Str_sMsg *msg);

#ifdef __cplusplus
}
#endif

#endif
