#ifndef VEC3_H
#define VEC3_H

#include <math.h>
#include <cstdlib>
#include <valarray>
#include <iostream>


class Vec3
{
 public:
    double v[3];
	double get(int i);
    Vec3(double xi=0,double yi=0,double zi=0);
    Vec3(std::valarray<double>* d);
    void set_elem(double xi,double yi,double zi);
    void print();
    void sum(Vec3 b, Vec3* s);
    void normalize();
    double abs();
    void cross(Vec3 b, Vec3* s);
    double dot(const Vec3& b);
    Vec3 cross(const Vec3& b);
    Vec3 elem_mul(Vec3 b);
    Vec3 operator+(const Vec3& b);
    Vec3 operator-(const Vec3& b);
    Vec3 operator-();
    Vec3 operator*(double k);
    Vec3 operator+(double b);
	Vec3& operator+=(const Vec3& b);
    
    friend std::istream &operator>>(std::istream &stream , Vec3 &a);
    friend std::ostream &operator<<(std::ostream &stream , Vec3 &a);
    double dist(const Vec3& b);

};

#endif

