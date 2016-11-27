#include "ground_vehicle_estimator.h"
#include "rotation.h"
#include "vec3.h"

void estimate_position(double px, double py, CameraMatrix &cm, double* gimbal_angles, double* gimbal_offset, double *ang, double *p_UAV, double h, double *g_pos)
{
  double a = (px - cm.cx) / cm.fx; // find direction to ground vehicle as (a,b,1) from camera matrix parameters
  double b = (py - cm.cy) / cm.fy;
  Rotation R(ang[0], ang[1], ang[2]); // rotation matrix
  Rotation gimbal_R(0, gimbal_angles[1], gimbal_angles[0]);
  Rotation combined_R = R * gimbal_R;
  Vec3 gimbal_offset_vec(gimbal_offset[0], gimbal_offset[1], gimbal_offset[2]);
  Vec3 v(a,b,1); // direction vector from camera towards ground vehicle
  Vec3 v1 = combined_R.transpose()*v;
  double Z = (h-gimbal_offset[2])/v1.v[2];
  Vec3 p_UAV_v(p_UAV[0], p_UAV[1], h);
  Vec3 p_ground_vehicle_v = p_UAV_v + v1*Z + R.transpose()*gimbal_offset_vec;
  g_pos[0] = p_ground_vehicle_v.v[0];
  g_pos[1] = p_ground_vehicle_v.v[1];
}
