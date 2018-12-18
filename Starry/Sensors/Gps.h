#ifndef __GPS_H
#define	__GPS_H

#include "sensor.h"

void GpsDataPreTreat(void);
void TransVelToBodyFrame(Vector3ForFloat velEf, Vector3ForFloat* velBf, float yaw);
void TransVelToEarthFrame(Vector3ForFloat velBf, Vector3ForFloat* velEf, float yaw);
float GetMagDeclination(void);
bool GpsGetFixStatus(void);
float GpsGetAccuracy(void);
Vector3ForFloat GpsGetVelocity(void);
Vector3ForFloat GpsGetPosition(void);

void GpsResetHomePosition(void);
void GpsTransToLocalPosition(Vector3ForFloat* position, double lat, double lon);

float GetDirectionToHome(Vector3ForFloat position);
float GetDistanceToHome(Vector3ForFloat position);
float GetDirectionOfTwoPoint(Vector3ForFloat point1, Vector3ForFloat point2);
Vector3ForFloat GetHomePosition(void);
void GetHomeLatitudeAndLongitude(double* lat, double* lon);

#endif















