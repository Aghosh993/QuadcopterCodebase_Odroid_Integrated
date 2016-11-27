#ifndef GIMBAL_CONTROLLER_H
#define GIMBAL_CONTROLLER_H

#include "detector_planner_interface.h"

class PlannerInputs;
class QR_Commands;

class GimbalController
{
public:
  GimbalController();
  void update(PlannerInputs &planner_inputs, QR_Commands &qr_commands);
};

#endif
