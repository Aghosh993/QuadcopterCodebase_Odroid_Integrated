
/*
 Author: Marcus Vinicius de Resende Maia Leite
 NYU Tandon School of Engineering Polytechnic Institute
 New York University 
 Control/Robotics Research Laboratory (CRRL)

 Code to test the Path Planning code for MBZIRC 2017. The test compreends of localizing and landing on a moving target within a 10 meter radius of the UAV.

 INPUTS
 - UGV 2D position coordinates
 - UGV 2D velocity 

 OUTPUTS
 - Path data
	path[n][0]: x coord of the n-th waypoint
	path[n][1]: y coord of the n-th waypoint
	path[n][2]: z coord of the n-th waypoint
	path[n][3]: desired top speed towards the n-th waypoint
	path[n][4]: desired x-axis top_speed towards the n-th waypoint
	path[n][5]: desired y-axis top_speed towards the n-th waypoint
	path[n][6]: desired z-axis top_speed towards the n-th waypoint
 - Land Flag
*/

#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <cmath>
//#include <rpi_comms.h>
//#include "serialport_linux.h"
#include "path_planner.h"
#include "detector_planner_interface.h"

// Choose frame of reference
//#define WORLD_FRAME_REFERENCE
#define LOCAL_FRAME_REFERENCE

// Globals
static int radius = 1; //m
static float top_uav_speed = 2; // Max UAV speed in m/s (8 m / s, approx 30 km / h(max for mbzirc))
static float top_vx = 2;
static float top_vy = 2;
static float top_vz = 0.2;
static float ugv_speed = 1;	// UGV speed in m/s
static int shut_down_engines = 0;	// Flag to land
static float back_offset = 2; // Offset time to come from behind the target in seconds
static float f_cam = 30; // 30 Hz
static int n_points_visited = 0; // Number of points visited
static std::vector<std::vector<float>> ugv_pos;

//////////////////////////////////////////////////////////////
// Function to import the search path data
// Input: csv file name
// Output : table of x, y coordinates
static std::vector<std::vector<float>> import_search(std::ifstream& search_file, std::string& file_name) {
	search_file.open(file_name);
	if (!search_file) {				// Checking if the file was opened
		std::cerr << "Error: Couldnt open search path file!\n";
		exit(1);
	}
	char a;
	float data;
	std::vector<std::vector<float>> search_path_data(3);
	for (int i = 0; i < search_path_data.size(); i++) {
		search_file >> data;
		search_path_data[i].push_back(data);
		search_file >> a;
		search_file >> data;
		search_path_data[i].push_back(data);
		search_file >> a;
		search_file >> data;
		search_path_data[i].push_back(data);
		search_file >> a;
		search_file >> data;
		search_path_data[i].push_back(data);
	}
	search_file.close();
	return search_path_data;
}

//Importing the uav SEARCH PATH
//static std::ifstream search_data;
static std::vector<std::vector<float>> path = {{0,0,0,0}};// = import_search(search_data, "test_search_path.txt");

//////////////////////////////////////////////////////////////
// Landing Path Planner Function to put the UAV behind the UGV
// Input: UAV's current position, UGV's current position, UGV's speed
// Output : The landing path data as described in the intro
static std::vector<std::vector<float>> landing_path_planner(std::vector<float>& UGV_pos, std::vector<float>& UAV_pos, std::vector<float>& UGV_speed) {
	//std::cout << "landing_path_planner\n";
	float dist = std::sqrt((UGV_pos[0])*(UGV_pos[0]) + (UGV_pos[1])*(UGV_pos[1]) + (UAV_pos[2]*UAV_pos[2]) );
	float t_ugv = dist;
	float t_uav = 0;
	float pred_s = t_ugv; //Initial value
	std::vector<float> dest, dest_prev;
	std::string curve_style;
	float sign;
	while ((t_ugv > t_uav + 0.1) || (t_ugv < t_uav - 0.1)) {
		if (t_ugv > t_uav) sign = 1;
		else sign = -1;
		pred_s = pred_s + sign / 10;
		//Predict destination position
		dest = {UGV_pos[0] + pred_s*UGV_speed[0], UGV_pos[1] + pred_s*UGV_speed[1], 2};
		dest_prev = {UGV_pos[0] + (float) (pred_s-back_offset)*UGV_speed[0], UGV_pos[1] + (float) (pred_s-back_offset)*UGV_speed[1]};
		t_ugv = pred_s ; //Estimate t_ugv based on the indexes
		//Estimate t_uav
		t_uav = std::sqrt((dest[0]-UAV_pos[0])*(dest[0]-UAV_pos[0]) + (dest[1]-UAV_pos[1])*(dest[1]-UAV_pos[1]) + (dest[2]-UAV_pos[2])*(dest[2]-UAV_pos[2])) / top_uav_speed;	
		//std::cout << t_ugv << std::endl << t_uav << std::endl;
	}

	if (pred_s < 0) pred_s = 0;
	//std::cout << "curve decision\n";
	// Condition to do the curve
	std::vector<float> positionVec = { dest[0]-UAV_pos[0], dest[1]-UAV_pos[1]};
	float positionVec_ang = (180 / 3.14)*std::atan2(positionVec[1], positionVec[0]); //Distance std::vector from UGV to UAV
	std::vector<float> speedVec_ugv = { UGV_speed[0], UGV_speed[1] };
	float speedVec_ugv_ang = (180 / 3.14)*std::atan2(speedVec_ugv[1], speedVec_ugv[0]); //Speed std::vector of UGV

	//Angle difference
	float ang_diff = std::abs(speedVec_ugv_ang - positionVec_ang);
	//Define curve to be used
	if (ang_diff < 60 || ang_diff>350) curve_style = "no_curve";
	else if (ang_diff < 140 && ang_diff>60) curve_style = "90_curve";
	else if (ang_diff>140 && ang_diff < 350) curve_style = "180_curve";
	// Update prediction time based on curve type
	if (curve_style == "180_curve") pred_s = pred_s + 3.14*radius / 8;
	else if (curve_style == "90_curve" || curve_style == "no_curve") pred_s = pred_s + 3.14*radius / 8;

	// Predict final destination position
	dest = {UGV_pos[0] + pred_s*UGV_speed[0],UGV_pos[1] + pred_s*UGV_speed[1]};
	dest_prev = {UGV_pos[0] + (float) (pred_s-back_offset)*UGV_speed[0],UGV_pos[1] + (float) (pred_s-back_offset)*UGV_speed[1]};
		
	//Curve Calculation
	float ugv_ang = std::atan2((-UGV_speed[1]), (-UGV_speed[0]));

	//Present Parallel Point
	float x2 = radius*std::sin(ugv_ang);
	float y2 = -radius*std::cos(ugv_ang);
	float z2 = 2;

	std::vector<float> x = {};
	std::vector<float> y = {};
	std::vector<float> z = {};

	//Come from right or left decision
	if (((UAV_pos[0]-(dest_prev[0]+x2))*(UAV_pos[0]-(dest_prev[0]+x2)) + (UAV_pos[1]-(dest_prev[1]+y2))*(UAV_pos[1]-(dest_prev[1]+y2))) > ((UAV_pos[0]-(dest_prev[0]-x2))*(UAV_pos[0]-(dest_prev[0]-x2))+(UAV_pos[1]-(dest_prev[1]-y2))*(UAV_pos[1]-(dest_prev[1]-y2)))) {
		x2 = -x2;
		y2 = -y2;

		// Circle fitting
		float r = radius / 2;
		for (float t = 1.5; t > -1.5 ; t = t - 0.3) {
			x.push_back(x2 / 2 + r*std::cos(t));
			y.push_back(y2 / 2 + r*std::sin(t));
			z.push_back(z2);
		}
	}
	else {
		//Circle fitting
		float r = radius / 2;
		for (float t = -1.5; t < 1.5; t = t + 0.3) {
			x.push_back(x2 / 2 + r*std::cos(t));
			y.push_back(y2 / 2 + r*std::sin(t));
			z.push_back(z2);
		}
	}
	std::vector<std::vector<float>> new_path = {};
	
	//std::cout << "draw curve\n";
	//Path data construction
	// Finish point
	new_path.push_back({ dest[0],dest[1],2, (float) 0.5*top_uav_speed });

	if (curve_style == "no_curve") {
		//std::cout << "no_curve\n";
		new_path.push_back({ dest_prev[0],dest_prev[1],2, top_uav_speed });
	}
	else if (curve_style == "180_curve") {
		//std::cout << "180_curve\n";
		for (int i = x.size()-1; i > 0; i--) {
			new_path.push_back({ dest_prev[0] + x[i],dest_prev[1] + y[i],z[i], top_uav_speed });
		}
	}
	else if (curve_style == "90_curve") {
		//std::cout << "90_curve\n";
		for (int i = (x.size()/2); i > 0; i--) {
			new_path.push_back({ dest_prev[0] + x[i],dest_prev[1] + y[i],z[i], top_uav_speed });
		}

	}
	return new_path;
}

/////////////////////////////////////////////////////////////
// Path Planner Driver
// Input: Flags from image processing and UAV's current position
// Output: Next waypoint to visit with x, y, x, v, vx, vy, vz
static int path_planner(bool first_detection, bool notfound, bool above_ugv, std::vector<float>& uav_pos, std::vector<float>& UGV_pos, std::vector<float>& UGV_speed){
	//	Set flags
	bool height_less0_2 = uav_pos[2] < 0.2 ? true : false;
	bool height_over1_5 = uav_pos[2] > 1.5 ? true : false;

	// UGV NOT DETECTED --> Search Path
	if (!first_detection) {
		// Case it reaches the end of the searching path
		if (path.size() == 0) { 
			//path = import_search(search_data, "test_search_path.txt"); 
			//path.pop_back();
		}
	}

	// UGV DETECTED --> Landing Path
	else if (first_detection && !height_less0_2 && !notfound) {
		// Get UGV global position and speed from image processing here
		
		if(abs(UGV_speed[0] + UGV_speed[1]) > 0.05){ // If target is moving
			float dist_uav_point = (uav_pos[0] - path[path.size() - 1][0])*(uav_pos[0] - path[path.size() - 1][0]) + (uav_pos[1] - path[path.size() - 1][1])*(uav_pos[1] - path[path.size() - 1][1]) + (uav_pos[2] - path[path.size() - 1][2])*(uav_pos[2] - path[path.size() - 1][2]);
			if ((path.size() < 2 || (dist_uav_point > 0.5*0.5)) && (UGV_speed[0]+UGV_speed[1]<10)) { // If it's not in the curve
			path = landing_path_planner(UGV_pos, uav_pos, UGV_speed);
			}
			else {
				#ifdef LOCAL_FRAME_REFERENCE
					float dx(path[path.size() - 1][0] - uav_pos[0]), dy(path[path.size() - 1][1] - uav_pos[1]), dz(path[path.size() - 1][2] - uav_pos[2]);
					float t = std::sqrt(dx*dx + dy*dy + dz*dz) / path[path.size() - 1][3];
					t = t == 0 ? 1 : t;
					float vx(dx / t), vy(dy / t), vz(dz / t);
					// Update path points with UAV velocity
					for (int i = 0; i < path.size(); i++) {
					path[i][0] -= vx * 1/f_cam;
					path[i][1] -= vy * 1/f_cam;
					}
				#endif
			}

			if(above_ugv){	// Come down and handle discontinuity
				while (path.size() > 1) path.pop_back();
				path[0][2] = 0.1;
				path[0][3] = 0.55*top_uav_speed;
			}
		}
		
		else { 	// If the target is not moving
				path[0][0]=UGV_pos[0]; 
				path[0][1]=UGV_pos[1]; 
				path[0][2]=2;
				path[0][3]=0.25*top_uav_speed;
				if (path.size() > 1) {
					while (path.size() > 2) path.pop_back();
					path[1][0] = UGV_pos[0];
					path[1][1] = UGV_pos[1];
					path[1][2] = 2;
					path[1][3] = 0.25*top_uav_speed;
				}
				else
				{
					path.push_back({ UGV_pos[0],UGV_pos[1],2, (float) 0.25*top_uav_speed });
				}

			if(above_ugv){	// Come down and handle discontinuity
				while (path.size() > 1) path.pop_back();
				path[0][2] = 0.1;
				path[0][3] = 0.1*top_uav_speed;

			}

		}

		/*float dist_uav_point = (uav_pos[0] - path[path.size() - 1][0])*(uav_pos[0] - path[path.size() - 1][0]) + (uav_pos[1] - path[path.size() - 1][1])*(uav_pos[1] - path[path.size() - 1][1]) + (uav_pos[2] - path[path.size() - 1][2])*(uav_pos[2] - path[path.size() - 1][2]);
		if (path.size()<3 || (dist_uav_point > 4)) { // If UAV is 2 m away from waypoint, update them all
			// Get UGV global position and speed from image processing here
			if(UGV_speed[0] + UGV_speed[1] > 0.05)landing_path_planner(UGV_pos, uav_pos, UGV_speed);
			else { 			// If the target is not moving
				path[0][0]=UGV_pos[0]; 
				path[0][1]=UGV_pos[1]; 
				path[0][2]=2;
				path[0][3]=0.15;
				path[1][0]=UGV_pos[0]; 
				path[1][1]=UGV_pos[1]; 
				path[1][2]=2;
				path[1][3]=0.15;			
			}
			if(above_ugv){	// Interatively come down
				above_n++;
				path[0][2] = path[0][2] - 0.5*above_n;
				path[1][2] = path[1][2] - 0.5*above_n;
				path[0][3] = path[0][3] + 0.15;
				path[1][3] = path[1][3] + 0.15;
			}
		}
		*/
	}

	// UGV MISSED AFTER DETECTING IT
	else if (first_detection && height_over1_5 && notfound) {
		// Continue last path
		// Slow down
		path[0][3] = path[0][3] / 2;
	}
	
	// UGV MISSED WHEN TRYING TO LAND
	else if(first_detection && !height_over1_5 && !height_less0_2 && notfound){
		// go up and try to detect again
		if(path[path.size()-1][2]<4) path[path.size()-1][2] = path[path.size()-1][2] + 0.3;
	}

	// Land
	else if(first_detection && !height_over1_5 && height_less0_2 && !notfound){
		// Set flag to land
		shut_down_engines = 1;
	}

	// Getting next waypoint
	float dist_uav_point = (uav_pos[0] - path[path.size() - 1][0])*(uav_pos[0] - path[path.size() - 1][0]) + (uav_pos[1] - path[path.size() - 1][1])*(uav_pos[1] - path[path.size() - 1][1]) + (uav_pos[2] - path[path.size() - 1][2])*(uav_pos[2] - path[path.size() - 1][2]);
	if (path.size()>1 && (dist_uav_point < 0.5*0.5)) n_points_visited++; // If UAV position to waypoint is less than 50cm mark as visited
	for (int i = n_points_visited; i > 0; i--) if(path.size()>1)path.pop_back();

	// Calculate speeds
	float dx(path[path.size() - 1][0] - uav_pos[0]), dy(path[path.size() - 1][1] - uav_pos[1]), dz(path[path.size() - 1][2] - uav_pos[2]);
	float t = std::sqrt(dx*dx + dy*dy + dz*dz) / path[path.size() - 1][3];
	t = t == 0 ? 1:t;
	float vx = dx/t > top_vx ? top_vx : dx/t;
	float vy = dy/t > top_vy ? top_vy : dy/t;
	float vz = dx/t > top_vz ? top_vz : dz/t;
	
	if (path[path.size() - 1].size()>5) {
		path[path.size() - 1][4]=vx;
		path[path.size() - 1][5]=vy;
		path[path.size() - 1][6]=vz;
	}
	else {
		path[path.size() - 1].push_back(vx);
		path[path.size() - 1].push_back(vy);
		path[path.size() - 1].push_back(vz);
	}

	//std::cout << "Path Points:\n";
	//for (int i = 0; i < path.size(); i++) std::cout << path[i][0] << " " << path[i][1] << " " << path[i][2] << std::endl;

	return 0;
}

PathPlanner::PathPlanner(){}
void PathPlanner::update(PlannerInputs &planner_inputs, QR_Commands &qr_commands)
{		
		//std::cout << "Path planner update begins\n";
		std::vector<float> UAV_pos(3), UGV_pos(2), UGV_speed(2);
		bool first_detection, not_found, above_ugv;
		
		// Get inputs
		#ifdef WORLD_FRAME_REFERENCE
		// UAV coords
		UAV_pos[0] = planner_inputs.qr_state->x;
		UAV_pos[1] = planner_inputs.qr_state->y;
		UAV_pos[2] = planner_inputs.qr_state->z;
		// UGV coords
		UGV_pos[0] = planner_inputs.detector_outputs->x_w;
		UGV_pos[1] = planner_inputs.detector_outputs->y_w;
		// UGV speed
		UGV_speed[0] = planner_inputs.detector_outputs->vx_w;
		UGV_speed[1] = planner_inputs.detector_outputs->vy_w;
		#endif

		#ifdef LOCAL_FRAME_REFERENCE
		// UAV coords
		UAV_pos[0] = 0.0;
		UAV_pos[1] = 0.0;
		UAV_pos[2] = planner_inputs.qr_state->z;
		// UGV coords
		UGV_pos[0] = planner_inputs.detector_outputs->x_b;
		UGV_pos[1] = planner_inputs.detector_outputs->y_b;
		// UGV speed
		UGV_speed[0] = planner_inputs.detector_outputs->vx_b;
		UGV_speed[1] = planner_inputs.detector_outputs->vy_b;
		#endif

	//	ugv_pos.push_back({UGV_pos[0],UGV_pos[1]});

	//if(ugv_pos.size()>3){

	//	UGV_speed[0] = ugv_pos[ugv_pos.size()-1][0]-ugv_pos[ugv_pos.size()-2][0]/(1/f_cam);
	//	UGV_speed[1] = ugv_pos[ugv_pos.size()-1][1]-ugv_pos[ugv_pos.size()-2][1]/(1/f_cam);

		//std::cout << " UGV Speed: " << UGV_speed[0] << "," << UGV_speed[1] <<std::endl;
		//std::cout << "UGV Position: " << UGV_pos[0] << "," << UGV_pos[1] <<std::endl;

		// CV flags
		first_detection = planner_inputs.detector_outputs->first_detection; 
		not_found = planner_inputs.detector_outputs->not_found; 
		above_ugv = false; planner_inputs.detector_outputs->above_ugv;

		// RUN PATH PLANNER
		path_planner(first_detection, not_found, above_ugv, UAV_pos, UGV_pos, UGV_speed);

		// Set outputs
		qr_commands.vx = path[path.size()-1][4];
		qr_commands.vy = path[path.size()-1][5];
		qr_commands.vz = path[path.size()-1][6];
		qr_commands.z = path[path.size()-1][2]; // Z (currently not used)
		qr_commands.land_command = shut_down_engines;
		qr_commands.motion_commands_updated = 1; // Commands updated
		// Print outputs
		//std::cout << "Path Planning Output\n vx: ";
		//std::cout << path[path.size() - 1][4] << std::endl;
		//std::cout << "\n vy: ";
		//std::cout << path[path.size() - 1][5] << std::endl;
		//std::cout << "\n vz: ";
		//std::cout << path[path.size() - 1][6] << std::endl;
		//std::cout << "\n z: ";
		//std::cout << path[path.size() - 1][2] << std::endl; // Z (currently not used)
		//std::cout << "\n landcommand: ";
		//std::cout << shut_down_engines << std::endl;
		
		// Send points to be plot
		// qr_commands.x_y_z_UAV_frame = path; // std::vector<std::vector<float>>
		// Create array of arrays

		//std::cout << "Path planner update ends\n";


		//plot_path(path, cm, uav_r, uav_p, uav_y, cam_p, cam_y );

	//}
		/* Send data to autopilot
		setup_linux_serial('/dev/ttyUSB0');
		uint8_t outgoing_data_buffer[sizeof(path_planner_out_data)+1];
		create_status_transmission(path[path.size()-1][4], path[path.size()-1][5], path[path.size()-1][6], path[path.size()-1][2], shut_down_engines, outgoing_data_buffer);
		for(int i = 0; i < sizeof(path_planner_out_data)+1;i++){
			uart_send_byte_linux(outgoing_data_buffer[i]);
		}*/
}