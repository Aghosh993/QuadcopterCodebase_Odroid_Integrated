#ifndef GROUND_VEHICLE_ESTIMATOR
#define GROUND_VEHICLE_ESTIMATOR

struct CameraMatrix
{
  double fx;
  double fy;
  double cx;
  double cy;
};

// px and py specify the pixel location of the ground vehicle (e.g., center of landing pad) in the camera image.
// cm is the camera matrix; the units used in specifying px and py should be consistent with the camera matrix.
// ang is a pointer to an array of length 3 and provides roll, pitch, and yaw measurements.
// p_UAV is a pointer to an array of length 2 and provides the UAV's (X,Y) location in some suitable fixed coordinate frame with Z axis vertically down.
// h is the height (altitude) of the UAV relative to the ground vehicle's reference point (e.g., center of landing pad)
// g_pos should be a pointer to an array of length 2; this function will fill in the elements of g_pos to be the X-axis and Y-axis coordinates of the ground vehicle with respect to the same coordinate frame used in specifying p_UAV.
// If it is desired to compute the relative position (i.e., position of the ground vehicle with respect to a world-aligned coordinate frame, but with origin at the location of the UAV), simply set the elements of p_UAV to 0
// Gimbal angles: array with gimbal heading and gimbal pitch. 0 degrees is with the camera pointing down
// Gimbal offset: from the center of the UAV to the camera
void estimate_position(double px, double py, CameraMatrix &cm, double* gimbal_angles, double* gimbal_offset, double *ang, double *p_UAV, double h, double *g_pos);

#endif
