#ifndef QUADROTOR_INTERFACE_H
#define QUADROTOR_INTERFACE_H

class QR_Commands;
void send_qr_commands(QR_Commands &qr_c);
int is_autopilot_ready();
void send_abort_command();

#endif
