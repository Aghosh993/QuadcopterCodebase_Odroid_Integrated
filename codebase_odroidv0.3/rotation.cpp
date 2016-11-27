#include "rotation.h"

Rotation::Rotation(double theta_x, double theta_y, double theta_z)
{
    double cos_halftheta0, sin_halftheta0, cos_halftheta1, sin_halftheta1, cos_halftheta2, sin_halftheta2, q0,q1,q2,q3;

    cos_halftheta0=cos(theta_x/2.0);
    sin_halftheta0=sin(theta_x/2.0);
    cos_halftheta1=cos(theta_y/2.0);
    sin_halftheta1=sin(theta_y/2.0);
    cos_halftheta2=cos(theta_z/2.0);
    sin_halftheta2=sin(theta_z/2.0);
    q0=cos_halftheta0*cos_halftheta1*cos_halftheta2+sin_halftheta0*sin_halftheta1*sin_halftheta2;
    q1=sin_halftheta0*cos_halftheta1*cos_halftheta2-cos_halftheta0*sin_halftheta1*sin_halftheta2;
    q2=cos_halftheta0*sin_halftheta1*cos_halftheta2+sin_halftheta0*cos_halftheta1*sin_halftheta2;
    q3=cos_halftheta0*cos_halftheta1*sin_halftheta2-sin_halftheta0*sin_halftheta1*cos_halftheta2;
    R[0][0]=q0*q0+q1*q1-q2*q2-q3*q3;
    R[0][1]= 2*(q1*q2-q0*q3);
    R[0][2]=2*(q1*q3+q0*q2);
    R[1][0]=2*(q1*q2+q0*q3);
    R[1][1]=q0*q0-q1*q1+q2*q2-q3*q3;
    R[1][2]=2*(q2*q3-q0*q1);
    R[2][0]=2*(q1*q3-q0*q2);
    R[2][1]=2*(q2*q3+q0*q1);
    R[2][2]= q0*q0-q1*q1-q2*q2+q3*q3;
}


Rotation::Rotation(const Mat3& m)
{
    for (int i=0; i<3; i++) {
        for (int j=0; j<3; j++) {
            R[i][j] = m.R[i][j];
        }
    }
}


void Rotation::set_axis_angle(Vec3 k, double theta)
{
    double kx = k.v[0];
    double ky = k.v[1];
    double kz = k.v[2];
    double ctheta = cos(theta);
    double stheta = sin(theta);
    double vtheta = 1-ctheta;
    R[0][0] = kx*kx*vtheta + ctheta;
    R[0][1] = kx*ky*vtheta - kz*stheta;
    R[0][2] = kx*kz*vtheta + ky*stheta;
    R[1][0] = kx*ky*vtheta + kz*stheta;
    R[1][1] = ky*ky*vtheta + ctheta;
    R[1][2] = ky*kz*vtheta - kx*stheta;
    R[2][0] = kx*kz*vtheta - ky*stheta;
    R[2][1] = ky*kz*vtheta + kx*stheta;
    R[2][2] = kz*kz*vtheta + ctheta;
}

Rotation Rotation::transpose()
{
    Rotation RT;
    int i,j;
    for (i=0;i<3;i++)
    {
	for(j=0;j<3;j++)
	{
	    RT.R[i][j]=R[j][i];
	}
    }
    return RT;
}


Vec3 Rotation::get_euler()
{
     return Vec3(atan2(R[2][1],R[2][2]), asin(-R[2][0]), atan2(R[1][0],R[0][0]));
}
