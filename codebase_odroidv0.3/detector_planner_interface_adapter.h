#ifndef DETECTOR_PLANNER_INTERFACE_ADAPTER_H
#define DETECTOR_PLANNER_INTERFACE_ADAPTER_H

class DetectorInputs;
class PlannerInputs;
class DetectorOutputs;

void update_qr_state();
void update_detector_inputs(DetectorInputs *detector_inputs);
void update_planner_inputs(PlannerInputs *planner_inputs, DetectorOutputs *detector_outputs);

#endif
