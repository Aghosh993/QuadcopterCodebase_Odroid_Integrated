#include "detector_planner_interface_adapter.h"
#include "detector_planner_interface.h"

extern QR_State qr_state;
//static QR_State qr_state1;

void update_detector_inputs(DetectorInputs *detector_inputs)
{
  detector_inputs->qr_state = &qr_state;
}

void update_planner_inputs(PlannerInputs *planner_inputs, DetectorOutputs *detector_outputs)
{
  planner_inputs->qr_state = &qr_state;
  planner_inputs->detector_outputs = detector_outputs;
}

void update_qr_state()
{

}
