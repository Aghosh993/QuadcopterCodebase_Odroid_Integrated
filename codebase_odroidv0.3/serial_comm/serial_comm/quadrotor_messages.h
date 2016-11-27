#ifndef QUADROTOR_MESSAGES_H
#define QUADROTOR_MESSAGES_H

#include <stdint.h>

#include "serial_comm/AP_msg_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#define QR_STATE_MSG_ID 41
#define QR_STATUS_MSG_ID 42
#define QR_MOTION_COMMAND_ID 51
#define QR_GIMBAL_COMMAND_ID 52
#define QR_LAND_COMMAND_ID 53


  typedef struct _QR_State_Msg
  {
    float x; // X-axis position
    float y; // Y-axis position
    float z; // Z-axis position
    float vx; // X-axis velocity
    float vy; // Y-axis velocity
    float vz; // Z-axis velocity
    float ang_x; // roll
    float ang_y; // pitch
    float ang_z; // yaw
    float angvel_x; // roll rate
    float angvel_y; // pitch rate
    float angvel_z; // yaw rate
    float t; // time
  } QR_State_Msg;

  typedef struct _QR_Status_Msg
  {
    uint32_t status;
  } QR_Status_Msg;

  typedef struct _QR_Motion_Command_Msg
  {
    float vx; // X-axis velocity command
    float vy; // Y-axis velocity command
	float vz; // Z-axis velocity (i.e., altitude rate) command
    float z;  // altitude command
    float ang_z; // yaw command
    float angvel_z; // yaw rate command
  } QR_Motion_Command_Msg;

  typedef struct _QR_Gimbal_Command_Msg
  {
    float pitch; // gimbal pitch command
    float heading_rate; // gimbal heading rate command
  } QR_Gimbal_Command_Msg;

  typedef struct _QR_Land_Command_Msg
  {
    uint32_t p; // dummy parameter -- in case we need to pass a parameter later on to customize the landing behavior
  } QR_Land_Command_Msg;

  void encode_qr_state_msg(QR_State_Msg *m, NFloats_sMsg *n_msg);
  void extract_qr_state_msg(QR_State_Msg *m, NFloats_sMsg *n_msg);
  void encode_qr_status_msg(QR_Status_Msg *m, NFloats_sMsg *n_msg);
  void extract_qr_status_msg(QR_Status_Msg *m, NFloats_sMsg *n_msg);
  void encode_qr_motion_command_msg(QR_Motion_Command_Msg *m, NFloats_sMsg *n_msg);
  void extract_qr_motion_command_msg(QR_Motion_Command_Msg *m, NFloats_sMsg *n_msg);
  void encode_qr_gimbal_command_msg(QR_Gimbal_Command_Msg *m, NFloats_sMsg *n_msg);
  void extract_qr_gimbal_command_msg(QR_Gimbal_Command_Msg *m, NFloats_sMsg *n_msg);
  void encode_qr_land_command_msg(QR_Land_Command_Msg *m, NFloats_sMsg *n_msg);
  void extract_qr_land_command_msg(QR_Land_Command_Msg *m, NFloats_sMsg *n_msg);
  int received_qr_state_msg(NFloats_sMsg *n_msg);
  int received_qr_status_msg(NFloats_sMsg *n_msg);
  int received_qr_motion_command_msg(NFloats_sMsg *n_msg);
  int received_qr_gimbal_command_msg(NFloats_sMsg *n_msg);
  int received_qr_land_command_msg(NFloats_sMsg *n_msg);
  void make_test_qr_state_msg(QR_State_Msg *m);
  void make_test_qr_status_msg(QR_Status_Msg *m);
  void make_test_qr_motion_command_msg(QR_Motion_Command_Msg *m);
  void make_test_qr_gimbal_command_msg(QR_Gimbal_Command_Msg *m);
  void make_test_qr_land_command_msg(QR_Land_Command_Msg *m);

#ifdef __cplusplus
}
#endif


#endif

