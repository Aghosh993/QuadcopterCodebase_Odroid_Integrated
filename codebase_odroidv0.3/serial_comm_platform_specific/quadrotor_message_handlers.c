#include "quadrotor_message_handlers.h"

int received_ready_from_autopilot = 0;


#define USE_PRINTF_DEBUG

#ifdef USE_PRINTF_DEBUG

#include <stdio.h>

void print_qr_state_msg(QR_State_Msg *m)
{
  printf("QR State Message: (%f %f %f) (%f %f %f) (%f %f %f) (%f %f %f)  \n", 
         m->x, m->y, m->z, m->vx, m->vy, m->vz,
         m->ang_x, m->ang_y, m->ang_z, m->angvel_x, m->angvel_y, m->angvel_z);
}

void print_qr_status_msg(QR_Status_Msg *m)
{
  printf("QR Status Message: %d\n", m->status);
}

void print_qr_motion_command_msg(QR_Motion_Command_Msg *m)
{
	printf("QR Motion Command Message: (%f %f %f) %f (%f %f)\n", 
         m->vx, m->vy, m->vz, m->z, m->ang_z, m->angvel_z);
}

void print_qr_gimbal_command_msg(QR_Gimbal_Command_Msg *m)
{
	printf("QR Gimbal Command Message: %f %f\n", m->pitch, m->heading_rate);
}

void print_qr_land_command_msg(QR_Land_Command_Msg *m)
{
	printf("QR Land Command Message: %d\n", m->p);
}

#endif


void process_qr_state_msg(QR_State_Msg *m)
{
  print_qr_state_msg(m);
}

void process_qr_status_msg(QR_Status_Msg *m)
{
  print_qr_status_msg(m);
  received_ready_from_autopilot = m->status;
}

void process_qr_motion_command_msg(QR_Motion_Command_Msg *m)
{

}

void process_qr_gimbal_command_msg(QR_Gimbal_Command_Msg *m)
{

}

void process_qr_land_command_msg(QR_Land_Command_Msg *m)
{

}

void process_nfloats_msg(NFloats_sMsg *msg)
{
  printf("Received nfloats message: ");
  for (int i=0; i<msg->n; i++) printf("%f ",msg->f[i]);
  printf("\n");
}

void process_str_msg(Str_sMsg *msg)
{
  printf("Received string message: ");
  for (int i=0; i<msg->n; i++) putchar(msg->s[i]);
  printf("\n");
}


