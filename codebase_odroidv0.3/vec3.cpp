#include <stdio.h>
#include "vec3.h"


void Vec3::print()
{
     printf("(%lf,%lf,%lf)",v[0],v[1],v[2]);
}

Vec3::Vec3(double xi,double yi,double zi)
{
     v[0]=xi;
     v[1]=yi;
     v[2]=zi;
}


Vec3::Vec3(std::valarray<double>* d)
{
    v[0] = (*d)[0];
    v[1] = (*d)[1];
    v[2] = (*d)[2];
}

void Vec3::set_elem(double xi,double yi,double zi)
{
     v[0]=xi;
     v[1]=yi;
     v[2]=zi;
}

void Vec3::sum(Vec3 b, Vec3* s)
{
     for(int i=0;i<3;i++) (*s).v[i]=v[i]+b.v[i];
}

void Vec3::normalize()
{
     double abs_p=sqrt(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]);
	 if (abs_p<1e-16){
		 return;
	 }
	 else{
		 v[0]/=abs_p;
		 v[1]/=abs_p;
		 v[2]/=abs_p;
	 }
}

double Vec3::abs()
{
     return sqrt(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]);
}

void Vec3::cross(Vec3 b, Vec3* s)
{
     (*s).v[0]=v[1]*b.v[2]-v[2]*b.v[1];
     (*s).v[1]=v[2]*b.v[0]-v[0]*b.v[2];
     (*s).v[2]=v[0]*b.v[1]-v[1]*b.v[0];
}

Vec3 Vec3::cross(const Vec3& b)
{
     Vec3 s(v[1]*b.v[2]-v[2]*b.v[1],v[2]*b.v[0]-v[0]*b.v[2],v[0]*b.v[1]-v[1]*b.v[0]);
     return s;
}

Vec3 Vec3::elem_mul(Vec3 b)
{
     Vec3 s(v[0]*b.v[0], v[1]*b.v[1], v[2]*b.v[2]);
     return s;
}

Vec3 Vec3::operator+(const Vec3& b)
{
     Vec3 s(v[0]+b.v[0],v[1]+b.v[1],v[2]+b.v[2]);
     return s;
}

Vec3 Vec3::operator-(const Vec3& b)
{
     Vec3 s(v[0]-b.v[0],v[1]-b.v[1],v[2]-b.v[2]);
     return s;
}

Vec3 Vec3::operator-()
{
     Vec3 s(-v[0],-v[1],-v[2]);
     return s;
}

Vec3 Vec3::operator*(double k)
{
  Vec3 s(k*v[0], k*v[1], k*v[2]);
  return s;
}

Vec3 Vec3::operator+(double b)
{
     Vec3 s(v[0]+b,v[1]+b,v[2]+b);
     return s;
}

Vec3& Vec3::operator+=(const Vec3& b)
{
    v[0] += b.v[0];
    v[1] += b.v[1];
    v[2] += b.v[2];
	return *this;
}

double Vec3::dist(const Vec3& b)
{
     return sqrt((v[0]-b.v[0])*(v[0]-b.v[0]) + (v[1]-b.v[1])*(v[1]-b.v[1]) + (v[2]-b.v[2])*(v[2]-b.v[2]));
}


double Vec3::dot(const Vec3& b)
{
    return v[0]*b.v[0] + v[1]*b.v[1] + v[2]*b.v[2]; 
}



std::istream &operator>>(std::istream &stream , Vec3 &a)
{
  stream >> a.v[0] >> a.v[1] >> a.v[2];
  return stream;
}


std::ostream &operator<<(std::ostream &stream , Vec3 &a)
{
  stream << a.v[0] << ' ' << a.v[1] << ' ' << a.v[2] << std::endl;
  return stream;
}

double Vec3::get(int i)
{
	return v[i];
}
