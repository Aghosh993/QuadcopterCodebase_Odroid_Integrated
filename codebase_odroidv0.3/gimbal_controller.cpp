#include "gimbal_controller.h"
#include "detector_planner_interface.h"

GimbalController::GimbalController()
{
}

void GimbalController::update ( PlannerInputs &planner_inputs, QR_Commands &qr_commands )
{
	static float pitch_Kp = 25.0 * 0.08;
	static float heading, pitch;

	static DetectorOutputs detector;
	static QR_State state;

	detector = *(planner_inputs.detector_outputs);
	state = *(planner_inputs.qr_state);

	heading = state.gimbal_heading;
	pitch = state.gimbal_pitch;

	/* Heading controller */
	// if( detector.px > 0.05 )
	// {
	// 	heading = 50;
	// }
	// else if ( detector.px < -0.05 )
	// {
	// 	heading = -50;
	// }
	// else
	// {
	// 	heading = 0;
	// }

	// qr_commands.gimbal_heading = heading;

	/* Pitch controller */
	if ( detector.py == 0 )
	{
		/* Update the pitch */
        pitch += ( detector.py ) * pitch_Kp;

        /* Check the pitch angle limits */
        if ( pitch > 60 )
        {
            pitch = 60;
        }
        else if ( pitch < -90 )
        {
            pitch = -90;
        }

        qr_commands.gimbal_pitch = pitch;
        qr_commands.gimbal_commands_updated = 1;

	}
}