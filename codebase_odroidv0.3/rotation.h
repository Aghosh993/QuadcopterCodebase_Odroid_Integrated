// implementation of a 3x3 rotation matrix and associated operations

#ifndef ROTATION_H
#define ROTATION_H

#include "mat3.h"


class Rotation : public Mat3
{
 public:
    Rotation(double theta_x=0,double theta_y=0,double theta_z=0);
    Rotation(const Mat3& m);
    void set_axis_angle(Vec3 k, double theta);
    Rotation transpose();
    Vec3 get_euler();
};

#endif
