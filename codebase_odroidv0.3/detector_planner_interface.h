#ifndef DETECTOR_PLANNER_INTERFACE_H
#define DETECTOR_PLANNER_INTERFACE_H

class QR_State
{
public:
  float x, y, z; // X, Y, Z position (world frame)
  float vx, vy, vz; // X, Y, Z velocities (world frame)
  float ang_x, ang_y, ang_z; // roll, pitch, yaw
  float angvel_x, angvel_y, angvel_z; // angular velocities (body frame)
  float q0, q1, q2, q3; // quaternion (corresponding to quadrotor roll, pitch, yaw)
  float t; // time
  float gimbal_pitch, gimbal_heading; // gimbal pitch and heading
};

class DetectorInputs
{
public:
  QR_State *qr_state;
};

class DetectorOutputs
{
public:
  float x_w, y_w; // X, Y of ground vehicle (world frame)
  float x_b, y_b; // X, Y of ground vehicle (UAV-aligned frame)
  float vx_w, vy_w; // X, Y velocity of ground vehicle (world frame)
  float vx_b, vy_b; // X, Y velocity of ground vehicle (UAV-aligned frame)
  float px, py; // X, Y pixel location of ground vehicle in camera image
  bool first_detection; // 0 before first detection of ground vehicle; 1 afterwards
  bool not_found; // 1 if UGV is not seen now (based on some number of previous frames)
  bool above_ugv; // 1 if UAV is above UGV
};

class PlannerInputs
{
public:
  QR_State *qr_state;
  DetectorOutputs *detector_outputs;
};

class QR_Commands
{
public:
  QR_Commands();
  float vx, vy, vz; // X, Y, Z velocities (world frame)
  float z; // Z (currently not used)
  float ang_z, angvel_z; // yaw, yaw velocity (currently not used -- holding fixed yaw)
  float gimbal_pitch, gimbal_heading_rate; // only gimbal_pitch currently used
  int motion_commands_updated; // set to 1 if updated (and therefore should be sent to inner-loop controller)
  int gimbal_commands_updated; // set to 1 if updated (and therefore should be sent to inner-loop controller)
  int land_command; // set to 1 to land
};


#endif
