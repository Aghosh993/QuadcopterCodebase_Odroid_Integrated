#include "detector_planner_interface.h"
#include "quadrotor_messages.h"
#include "quadrotor_interface.h"
#include "quadrotor_comm.h"

extern bool received_ready_from_autopilot;


void send_qr_commands(QR_Commands &qr_c)
{
  if (qr_c.land_command == 1)
  {
    QR_Land_Command_Msg m;
    send_land_command(&m);
  }
  if (qr_c.gimbal_commands_updated == 1)
  {
		QR_Gimbal_Command_Msg m;
    m.pitch = qr_c.gimbal_pitch;
    m.heading_rate = qr_c.gimbal_heading_rate;
    send_gimbal_command(&m);
  }
  if (qr_c.motion_commands_updated == 1)
  {
    QR_Motion_Command_Msg m;
    m.vx = qr_c.vx;
    m.vy = qr_c.vy;
    m.vz = qr_c.vz;
    m.z = qr_c.z;
    m.ang_z = qr_c.ang_z;
    m.angvel_z = qr_c.angvel_z;
    send_motion_command(&m);
  }
}

int is_autopilot_ready()
{
  return received_ready_from_autopilot;
}

void send_abort_command()
{
  QR_Land_Command_Msg m;
  send_land_command(&m);
}
