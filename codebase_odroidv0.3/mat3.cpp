#include <stdio.h>
#include "mat3.h"
#include "vec3.h"

#define epsilon 1e-15


Mat3::Mat3(Vec3 col0, Vec3 col1, Vec3 col2)
{
     R[0][0] = col0.v[0];  R[1][0] = col0.v[1];  R[2][0] = col0.v[2];
     R[0][1] = col1.v[0];  R[1][1] = col1.v[1];  R[2][1] = col1.v[2];
     R[0][2] = col2.v[0];  R[1][2] = col2.v[1];  R[2][2] = col2.v[2];
}

Mat3::Mat3()
{
     R[0][0] = 1;  R[1][0] = 0;  R[2][0] = 0;
     R[0][1] = 0;  R[1][1] = 1;  R[2][1] = 0;
     R[0][2] = 0;  R[1][2] = 0;  R[2][2] = 1;
}

Mat3::Mat3(Vec3 s)
{
     R[0][0] = 0;     R[1][0] = -s.v[2]; R[2][0] = s.v[1];
     R[0][1] = s.v[2];  R[1][1] = 0;     R[2][1] = -s.v[0];
     R[0][2] = -s.v[1]; R[1][2] = s.v[0];  R[2][2] = 0;
}

Mat3 Mat3::transpose()
{
     Mat3 RT;
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

Vec3 Mat3::operator*(Vec3 a)
{
     Vec3 s(0,0,0);
     for(int i=0;i<3;i++)
     {
	  for(int j=0;j<3;j++)
	  {
	       s.v[i]+=R[i][j]*a.v[j];
	  }
     }
     return s;
}

Mat3 Mat3::operator*(const Mat3& a)
{
     Mat3 s;
     for(int i=0;i<3;i++)
     {
	  for(int j=0; j<3; j++)
	  {
	       s.R[i][j] = 0;
	       for(int k=0; k<3; k++){
		    s.R[i][j] += R[i][k]*a.R[k][j];
	       }
	  }
     }
     return s;
}

Mat3 Mat3::operator+(const Mat3& a)
{
     Mat3 s;
     for(int i=0;i<3;i++)
     {
	  for(int j=0; j<3; j++)
	  {
	       s.R[i][j] = R[i][j] + a.R[i][j];
	  }
     }
     return s;
}

Mat3 Mat3::operator*(double d)
{
    Mat3 s;
    for(int i=0;i<3;i++)
    {
        for(int j=0; j<3; j++)
        {
            s.R[i][j] = R[i][j] * d;
        }
    }
    return s;
}


Mat3 Mat3::operator -()
{
    Mat3 s;
    for(int i=0;i<3;i++)
     {
	  for(int j=0; j<3; j++)
	  {
	       s.R[i][j] = -R[i][j];
	  }
     }
     return s;
}

void Mat3::print()
{
     for(int i=0; i<3; i++) printf("(%lf,%lf,%lf)\n",R[i][0],R[i][1],R[i][2]);
}

double Mat3::det()
{
     return R[0][0]*(R[1][1]*R[2][2]-R[2][1]*R[1][2]) 
	  - R[0][1]*(R[1][0]*R[2][2]-R[2][0]*R[1][2])
	  + R[0][2]*(R[1][0]*R[2][1]-R[2][0]*R[1][1]);
}

Mat3 Mat3::inverse()
{
     Mat3 Rinv;
     double Rdet=this->det();
     if (Rdet < epsilon) throw "Mat3::inverse -- Singularity encountered in matrix to be inverted";
     Rinv.R[0][0] = (R[1][1]*R[2][2]-R[1][2]*R[2][1])/Rdet;
     Rinv.R[0][1] = (R[0][2]*R[2][1]-R[0][1]*R[2][2])/Rdet;
     Rinv.R[0][2] = (R[0][1]*R[1][2]-R[0][2]*R[1][1])/Rdet;
     Rinv.R[1][0] = (R[1][2]*R[2][0]-R[1][0]*R[2][2])/Rdet;
     Rinv.R[1][1] = (R[0][0]*R[2][2]-R[0][2]*R[2][0])/Rdet;
     Rinv.R[1][2] = (R[0][2]*R[1][0]-R[0][0]*R[1][2])/Rdet;
     Rinv.R[2][0] = (R[1][0]*R[2][1]-R[1][1]*R[2][0])/Rdet;
     Rinv.R[2][1] = (R[0][1]*R[2][0]-R[0][0]*R[2][1])/Rdet;
     Rinv.R[2][2] = (R[0][0]*R[1][1]-R[0][1]*R[1][0])/Rdet;
     return Rinv;
}

void Mat3::set_as_Jr(double ang_x, double ang_y, double ang_z)
{
    double sx = sin(ang_x);
    double cx = cos(ang_x);
    double cy = cos(ang_y);
    double ty = tan(ang_y);
    R[0][0] = 1;
    R[0][1] = sx*ty;
    R[0][2] = cx*ty;
    R[1][0] = 0;
    R[1][1] = cx;
    R[1][2] = -sx;
    R[2][0] = 0;
    R[2][1] = sx/cy;
    R[2][2] = cx/cy;
}

void Mat3::set_all(double d)
{
    for (int i=0; i<3; i++)
    {
        for (int j=0; j<3; j++)
        {
            R[i][j] = d;
        }
    }
}

std::istream &operator>>(std::istream &stream , Mat3 &a)
{
  for (int i=0; i<3; i++)
  {
    for (int j=0; j<3; j++)
    {
    stream >> a.R[i][j];
    }
  }
  return stream;
}


std::ostream &operator<<(std::ostream &stream , Mat3 &a)
{
    for (int i=0; i<3; i++)
  {
    for (int j=0; j<3; j++)
    {
    stream << a.R[i][j] << ' ';
    }
    stream << std::endl;
  }
  stream << std::endl;
  return stream;
}

Mat3 mat3_diag(Vec3 s)
{
    Mat3 m;
    m.R[0][0] = s.v[0];
    m.R[1][1] = s.v[1];
    m.R[2][2] = s.v[2];
    return m;
}

Mat3 mat3_zeros()
{
    Mat3 m;
    m.set_all(0);
    return m;
}
