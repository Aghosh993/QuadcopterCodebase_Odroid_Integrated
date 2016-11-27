#include "quadrotor_messages.h"
#include "serial_comm/serial_datatypes.h"


void encode_qr_state_msg(QR_State_Msg *m, NFloats_sMsg *n_msg)
{
  n_msg->n = 14;
  n_msg->f[0] = interpret_int32_as_float(QR_STATE_MSG_ID);
  n_msg->f[1] = m->x;
  n_msg->f[2] = m->y;
  n_msg->f[3] = m->z;
  n_msg->f[4] = m->vx;
  n_msg->f[5] = m->vy;
  n_msg->f[6] = m->vz;
  n_msg->f[7] = m->ang_x;
  n_msg->f[8] = m->ang_y;
  n_msg->f[9] = m->ang_z;
  n_msg->f[10] = m->angvel_x;
  n_msg->f[11] = m->angvel_y;
  n_msg->f[12] = m->angvel_z;
  n_msg->f[13] = m->t;
}

void extract_qr_state_msg(QR_State_Msg *m, NFloats_sMsg *n_msg)
{
	m->x = n_msg->f[1];
	m->y = n_msg->f[2];
	m->z = n_msg->f[3];
	m->vx = n_msg->f[4];
	m->vy = n_msg->f[5];
	m->vz = n_msg->f[6];
	m->ang_x = n_msg->f[7];
	m->ang_y = n_msg->f[8];
	m->ang_z = n_msg->f[9];
	m->angvel_x = n_msg->f[10];
	m->angvel_y = n_msg->f[11];
	m->angvel_z = n_msg->f[12];
  m->t = n_msg->f[13];
}

void encode_qr_status_msg(QR_Status_Msg *m, NFloats_sMsg *n_msg)
{
  n_msg->n = 2;
  n_msg->f[0] = interpret_int32_as_float(QR_STATUS_MSG_ID);
  n_msg->f[1] = interpret_uint32_as_float(m->status);
}

void extract_qr_status_msg(QR_Status_Msg *m, NFloats_sMsg *n_msg)
{
	m->status = interpret_float_as_uint32(n_msg->f[1]);
}

void encode_qr_motion_command_msg(QR_Motion_Command_Msg *m, NFloats_sMsg *n_msg)
{
	n_msg->n = 7;
	n_msg->f[0] = interpret_int32_as_float(QR_MOTION_COMMAND_ID);
	n_msg->f[1] = m->vx;
	n_msg->f[2] = m->vy;
	n_msg->f[3] = m->vz;
	n_msg->f[4] = m->z;
	n_msg->f[5] = m->ang_z;
	n_msg->f[6] = m->angvel_z;
}

void extract_qr_motion_command_msg(QR_Motion_Command_Msg *m, NFloats_sMsg *n_msg)
{
	m->vx = n_msg->f[1];
	m->vy = n_msg->f[2];
	m->vz = n_msg->f[3];
	m->z = n_msg->f[4];
	m->ang_z = n_msg->f[5];
	m->angvel_z = n_msg->f[6];
}

void encode_qr_gimbal_command_msg(QR_Gimbal_Command_Msg *m, NFloats_sMsg *n_msg)
{
	n_msg->n = 3;
	n_msg->f[0] = interpret_int32_as_float(QR_GIMBAL_COMMAND_ID);
	n_msg->f[1] = m->pitch;
	n_msg->f[2] = m->heading_rate;
}

void extract_qr_gimbal_command_msg(QR_Gimbal_Command_Msg *m, NFloats_sMsg *n_msg)
{
	m->pitch = n_msg->f[1];
	m->heading_rate = n_msg->f[2];
}

void encode_qr_land_command_msg(QR_Land_Command_Msg *m, NFloats_sMsg *n_msg)
{
	n_msg->n = 2;
	n_msg->f[0] = interpret_int32_as_float(QR_LAND_COMMAND_ID);
	n_msg->f[1] = m->p;
}

void extract_qr_land_command_msg(QR_Land_Command_Msg *m, NFloats_sMsg *n_msg)
{
	m->p = n_msg->f[1];
}

int received_qr_state_msg(NFloats_sMsg *n_msg)
{
	QR_State_Msg m;
	extract_qr_state_msg(&m, n_msg);
	return 0;
}

int received_qr_status_msg(NFloats_sMsg *n_msg)
{
	QR_Status_Msg m;
	extract_qr_status_msg(&m, n_msg);
	return 0;
}

int received_qr_motion_command_msg(NFloats_sMsg *n_msg)
{
	QR_Motion_Command_Msg m;
	extract_qr_motion_command_msg(&m, n_msg);
	return 0;
}

int received_qr_gimbal_command_msg(NFloats_sMsg *n_msg)
{
	QR_Gimbal_Command_Msg m;
	extract_qr_gimbal_command_msg(&m, n_msg);
	return 0;
}

int received_qr_land_command_msg(NFloats_sMsg *n_msg)
{
	QR_Land_Command_Msg m;
	extract_qr_land_command_msg(&m, n_msg);
	return 0;
}

void make_test_qr_state_msg(QR_State_Msg *m)
{
	m->x = 1.1;
	m->y = -1;
	m->z = 2;
	m->vx = 1.5;
	m->vy = -1.67;
	m->vz = 2.22;
	m->ang_x = 0.1;
	m->ang_y = 0.2;
	m->ang_z = 0.3;
	m->angvel_x = 2.3;
	m->angvel_y = 3.3;
	m->angvel_z = 4.3;
}

void make_test_qr_status_msg(QR_Status_Msg *m)
{
	m->status = 1;
}

void make_test_qr_motion_command_msg(QR_Motion_Command_Msg *m)
{
	m->vx = 5.1;
	m->vy = 2.2;
	m->vz = 3.3;
	m->z = 5;
	m->ang_z = 1;
	m->angvel_z = -1.5;
}

void make_test_qr_gimbal_command_msg(QR_Gimbal_Command_Msg *m)
{
	m->pitch = 2.2;
	m->heading_rate = 5.5;
}

void make_test_qr_land_command_msg(QR_Land_Command_Msg *m)
{
	m->p = 2;
}
