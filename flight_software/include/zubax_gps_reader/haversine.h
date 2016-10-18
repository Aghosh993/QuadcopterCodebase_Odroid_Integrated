#ifndef HAVERSINE_H
#define HAVERSINE_H

#define PI 3.141592653589

double square(double a);
double haversine_distance_radians(double altitude, double latitude1, double longitude1, double latitude2, double longitude2);
double haversine_distance_degrees(double altitude, double latitude1, double longitude1, double latitude2, double longitude2);

#endif