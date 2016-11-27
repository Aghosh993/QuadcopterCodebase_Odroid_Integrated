#ifdef ENABLE_SERIAL_COMM
#include "Serial_USB.h"
#include "serial_comm/quadrotor_comm.h"
#include "quadrotor_interface.h"
#endif
#include "detector.h"
#include "path_planner.h"
#include "gimbal_controller.h"
#include "detector_planner_interface.h"
#include "detector_planner_interface_adapter.h"

#include <signal.h>
#include <fstream>

int run_prog;
static void sig_ctrlC_handler(int dummy)
{
  run_prog=0;
}
QR_State qr_state;



int main ()
{

  run_prog=1;
  signal(SIGINT,sig_ctrlC_handler);
  std::ofstream outfile;

  outfile.open("/home/alarm/data_collection/image_data.csv");

#ifdef ENABLE_SERIAL_COMM
  init_serial_usb_comm();
  init_serial_comm_datastructures();
  register_comm_callbacks();
#endif
  Detector detector;
  PathPlanner planner;
  GimbalController gimbal_controller;
  PlannerInputs planner_inputs;
  DetectorInputs detector_inputs;
  DetectorOutputs detector_outputs;


#ifdef ENABLE_SERIAL_COMM
  while (!is_autopilot_ready()) {}
#endif
  while (run_prog)
  {
#ifdef ENABLE_SERIAL_COMM
    update_serial_usb_comm();
    if (!is_autopilot_ready()) { send_abort_command(); break; }
#endif
    QR_Commands qr_commands;
    update_qr_state();
    qr_state.ang_x = 0;
    qr_state.z = 2;
    qr_state.ang_y = 0;
    qr_state.ang_z = 0;
    update_detector_inputs(&detector_inputs);
    detector.update(detector_inputs, detector_outputs);
    //outfile<<detector_outputs.px<<','<<detector_outputs.py<<','<<detector_outputs.vx_b<<','<<detector_outputs.vy_b<<','<<detector_outputs.x_b<<','<<detector_outputs.y_b<<','<<qr_commands.vx<<','<<qr_commands.vy<<','<<qr_commands.vz<<','<<qr_commands.land_command<<','<<detector_outputs.first_detection<<','<<detector_outputs.not_found<<std::endl;
    update_planner_inputs(&planner_inputs, &detector_outputs);
    planner.update(planner_inputs, qr_commands);
    gimbal_controller.update(planner_inputs, qr_commands);
#ifdef ENABLE_SERIAL_COMM
    send_qr_commands(qr_commands);
    flush_txbuf_comm();
#endif
  }
outfile.close();
}