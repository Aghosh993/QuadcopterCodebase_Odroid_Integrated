/*
Author: Marcus Vinicius de Resende Maia Leite
NYU Tandon School of Engineering Polytechnic Institute
New York University 
Control/Robotics Research Laboratory (CRRL)

Code to test the Path Planning code for MBZIRC 2017. The test compreends of localizing and landing on a moving target within a 10 meter radius of the UAV.

INPUTS
UGV 2D position coordinates
UGV 2D velocity 

OUTPUTS
Path data
	path[n][0]: x coord of the n-th waypoint
	path[n][1]: y coord of the n-th waypoint
	path[n][2]: z coord of the n-th waypoint
	path[n][3]: desired top speed towards the n-th waypoint
	path[n][4]: desired x-axis top_speed towards the n-th waypoint
	path[n][5]: desired y-axis top_speed towards the n-th waypoint
	path[n][6]: desired z-axis top_speed towards the n-th waypoint
*/

#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <cmath>

// Globals
int radius = 2; //m
float top_uav_speed = 2; // 8 m / s, approx 30 km / h(max for mbzirc)
float ugv_speed = 1;

//Function to import the search path data
// input: csv file name
// Output : table of x, y coordinates
std::vector<std::vector<float>> import_search(std::ifstream& search_file, std::string file_name) {
	search_file.open(file_name);
	if (!search_file) {				// Checking if the file was opened
		std::cerr << "Error: Couldnt open search path file!\n";
		exit(1);
	}
	char a;
	float data;
	std::vector<std::vector<float>> search_path_data(5);
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
std::ifstream search_data;
std::vector<std::vector<float>> search_path_data = import_search(search_data, "test_search_path.txt");

// Initialize the path
std::vector<std::vector<float>> path = search_path_data;

////////////////////////////////////////////////////////////// 
//Function to predict where the UGV will be
// Input: UAV's current position, UGV's current position, UGV's speed
// Output : The landing path data as described in the intro
std::vector<std::vector<float>> predict(std::vector<float> UAV_pos, std::vector<float> UGV_pos, std::vector<float> UGV_speed) {
	std::cout << "predict\n";
	float dist = std::sqrt((UGV_pos[0])*(UGV_pos[0]) + (UGV_pos[1])*(UGV_pos[1]));
	float t_ugv = dist;
	float t_uav = 0;
	float pred_s = t_ugv; //Initial value
	std::vector<float> dest, dest_prev;
	std::string curve_style;
	float sign;
	while ((t_ugv > t_uav + 0.25) || (t_ugv < t_uav - 0.25)) {
		if (t_ugv > t_uav) sign = -1;
		else sign = 1;
		pred_s = pred_s + sign / 10;
		//Predict destination position
		dest = {UGV_pos[0] + pred_s*UGV_speed[0], UGV_pos[1] + pred_s*UGV_speed[1], 2};
		dest_prev = {UGV_pos[0] + (float) (pred_s-0.5)*UGV_speed[0], UGV_pos[1] + (float) (pred_s-0.5)*UGV_speed[1]};
		t_ugv = pred_s ; //Estimate t_ugv based on the indexes
		//Estimate t_uav
		t_uav = std::sqrt((dest[0])*(dest[0]) + (dest[1])*(dest[1]) + (dest[2])*(dest[2])) / top_uav_speed;	
		std::cout << t_ugv << std::endl << t_uav << std::endl;
	}

	if (pred_s < 0) pred_s = 0;
	std::cout << "curve decision\n";
	// Condition to do the curve
	std::vector<float> positionVec = { dest[0], dest[1]};
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
	dest_prev = {UGV_pos[0] + (float) (pred_s-0.5)*UGV_speed[0],UGV_pos[1] + (float) (pred_s-0.5)*UGV_speed[1]};
		
	//Curve Calculation
	float ugv_ang = std::atan2((-UGV_speed[1]), (-UGV_speed[0]));
	//Parallel Point Besides UGV
	float x1 = radius*std::sin(ugv_ang);
	float y1 = -radius*std::cos(ugv_ang);
	//Present Parallel Point
	float x2 = radius*std::sin(ugv_ang);
	float y2 = -radius*std::cos(ugv_ang);
	float z2 = 2;

	std::vector<float> x = {};
	std::vector<float> y = {};
	std::vector<float> z = {};

	//Come from right or left decision
	if (((dest_prev[0] + x2)*(dest_prev[0] + x2) + (dest_prev[1] + y2)*(dest_prev[1] + y2)) > ((dest_prev[0] - x2)*(dest_prev[0] - x2) + (dest_prev[1] - y2)*(dest_prev[1] - y2))) {
		x1 = -x1;
		y1 = -y1;
		x2 = -x2;
		y2 = -y2;

		// Circle fitting
		float r = radius / 2;
		float x3 = x2 / 2 + r;
		float y3 = y2 / 2;
		float i = 0;
		for (float t = 1.5; t > -1.5 ; t = t - 0.3) {
			x.push_back(x2 / 2 + r*std::cos(t));
			y.push_back(y2 / 2 + r*std::sin(t));
			z.push_back(z2);
		}
	}
	else {
		//Circle fitting
		float r = radius / 2;
		float x3 = x2 / 2 + r;
		float y3 = y2 / 2;
		float i = 0;
		for (float t = -1.5; t < 1.5; t = t + 0.3) {
			x.push_back(x2 / 2 + r*std::cos(t));
			y.push_back(y2 / 2 + r*std::sin(t));
			z.push_back(z2);
		}
	}
	std::vector<std::vector<float>> new_path = {};
	
	std::cout << "draw curve\n";
	//Path data construction
	// Finish point
	new_path.push_back({ dest[0],dest[1],2,5 });

	if (curve_style == "no_curve") {
		std::cout << "no_curve\n";
		new_path.push_back({ dest_prev[0],dest_prev[1],2,2 });
	}
	else if (curve_style == "180_curve") {
		std::cout << "180_curve\n";
		for (int i = x.size()-1; i > 0; i--) {
			new_path.push_back({ dest_prev[0] + x[i],dest_prev[1] + y[i],z[i],2 });
		}
	}
	else if (curve_style == "90_curve") {
		std::cout << "90_curve\n";
		for (int i = (x.size()/2); i > 0; i--) {
			new_path.push_back({ dest_prev[0] + x[i],dest_prev[1] + y[i],z[i],2 });
		}

	}
	return new_path;
}

/////////////////////////////////////////////////////////////
// Landing Path Planner Function to put the UAV behind the UGV
// Input: UGV position and UAV position
// Output : Path points to visit
int landing_path_planner(std::vector<float> ugv_pos, std::vector<float> uav_pos, std::vector<float> ugv_speed) {
	std::cout << "landing_path_planner" << std::endl;
	//Predict
	path = predict(uav_pos, ugv_pos, ugv_speed);
	
	return 0;
}

/////////////////////////////////////////////////////////////
// Path Planner Driver
// Input: Flags from image processing and UAV's current position
// Output: Next waypoint to visit with x, y, x, v, vx, vy, vz
int path_planner(bool first_detection, bool notfound, std::vector<float> uav_pos){
	//	Set flags
	bool height_less0_2 = uav_pos[2] < 0.2 ? true : false;
	bool height_over1_5 = uav_pos[2] > 1.5 ? true : false;

	// UGV NOT DETECTED --> Search Path
	if (!first_detection) {
		// Case it reaches the end of the searching path
		if (path.size() == 0) { 
			path = search_path_data; 
			path.pop_back();
			path.pop_back();
			path.pop_back();
		}
		// Wait 1 sec
	}

	// UGV DETECTED --> Landing Path
	else if (first_detection && height_over1_5 && !notfound) {
		float dist_uav_point = (uav_pos[0] - path[path.size() - 1][0])*(uav_pos[0] - path[path.size() - 1][0]) + (uav_pos[1] - path[path.size() - 1][1])*(uav_pos[1] - path[path.size() - 1][1]) + (uav_pos[2] - path[path.size() - 1][2])*(uav_pos[2] - path[path.size() - 1][2]);
		if (path.size()>1 && (dist_uav_point > 4)) { // If UAV is 2 m away from waypoint, update them all
			// Get UGV global position and speed from image processing here
			landing_path_planner(UGV_pos, uav_pos, UGV_speed);
		}
		//	Wait 1 sec
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
		path[path.size()-1][2] = path[path.size()-1][2] + 0.3;
	}

	// Land
	else if(first_detection && !height_over1_5 && height_less0_2 && !notfound){
		// Set flag to land
	}

	// Getting next waypoint
	float dist_uav_point = (uav_pos[0] - path[path.size() - 1][0])*(uav_pos[0] - path[path.size() - 1][0]) + (uav_pos[1] - path[path.size() - 1][1])*(uav_pos[1] - path[path.size() - 1][1]) + (uav_pos[2] - path[path.size() - 1][2])*(uav_pos[2] - path[path.size() - 1][2]);
	if (path.size()>1 && (dist_uav_point < 1)) path.pop_back(); // If UAV position to waypoint is less than 1m move to next waypoint
	
	// Calculate speeds
	float dx(path[path.size()-1][0] - uav_pos[0]), dy(path[path.size() - 1][1] - uav_pos[1]), dz(path[path.size() - 1][2] - uav_pos[2]);
	float t = std::sqrt(dx*dx+dy*dy+dz*dz) / path[path.size() - 1][3];
	float vx(dx/t), vy(dy/t), vz(dz/t);
	path[path.size() - 1].push_back(vx);
	path[path.size() - 1].push_back(vy);
	path[path.size() - 1].push_back(vz);	

	for (int i = 0; i < path.size(); i++) std::cout << path[i][0] << " " << path[i][1] << " " << path[i][2] << std::endl;

	return 0;
}

int main(int argc,char* argv[])
{
	// Get CV flags
	// Get UAV position	
	path_planner(first_detection, not_found, UAV_pos);
	
	return(0);
}