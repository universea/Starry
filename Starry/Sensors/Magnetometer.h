#ifndef __MAGNETOMETER_H
#define	__MAGNETOMETER_H

#include "sensor.h"

typedef struct {
    Vector3ForFloat data;
    float mag;
    SENSOR_CALI_t cali;
    float earthMag;

} MAGNETOMETER_t;

void MagCaliDataInit(void);
void MagDataPreTreat(void);
void MagCalibration(void);
Vector3ForFloat MagGetData(void);
void MagCalibrateEnable(void);

Vector3ForFloat GetMagOffsetCaliData(void);
Vector3ForFloat GetMagScaleCaliData(void);

#endif



