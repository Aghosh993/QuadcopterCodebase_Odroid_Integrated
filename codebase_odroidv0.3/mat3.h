// implementation of a 3x3 matrix

#ifndef MAT3_H
#define MAT3_H

#include "vec3.h"


class Mat3 
{
public:
    double R[3][3];
    Mat3(Vec3 col0, Vec3 col1, Vec3 col2);
    Mat3(); //identity matrix
    Mat3(Vec3 s); //skew-symmetric matrix of s
    Mat3 transpose();
    Mat3 inverse();
    void set_as_Jr(double ang_x, double ang_y, double ang_z);
    Vec3 operator*(Vec3 a);
    Mat3 operator*(double d);
    Mat3 operator*(const Mat3& a);
    Mat3 operator+(const Mat3& a);
    Mat3 operator-();
    double det();
    void print();
    void set_all(double d);
    friend std::istream &operator>>(std::istream &stream , Mat3 &a);
    friend std::ostream &operator<<(std::ostream &stream , Mat3 &a);
};

Mat3 mat3_diag(Vec3 s);
Mat3 mat3_zeros();

#endif
