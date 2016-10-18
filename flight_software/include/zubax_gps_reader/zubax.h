#ifndef ZUBAX_H
#define ZUBAX_H 1

#include <stdio.h>

#include "NMEA.h"
#include "haversine.h"

#define SAMPLES 		100
#define ZUBAX_UART_PORT	UART5

enum states {
	COLLECTING_HOME_POINTS,
	EVALUATE_POINTS
};

typedef struct
{
	double homeLatitude;
	double homeLongitude;
	double homeAltitude;

	double x;
	double y;
	double z;

	double speed_x;
	double speed_y;

	uint32_t timestamp;
} gps_distance;

void gps_print_message(gps_data data, enum nmea_message_type desired_message);
void gps_print_data(gps_data data);

gps_distance gps_get_distance_from_origin(void);

#endif