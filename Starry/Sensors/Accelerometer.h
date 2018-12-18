#ifndef __ACCELEROMETER_H
#define	__ACCELEROMETER_H

#include "sensor.h"

typedef struct {
    Vector3ForFloat data;
    Vector3ForFloat dataLpf;
    float mag;
    float vibraCoef;
    LPF2ndData_t lpf_2nd;
    SENSOR_CALI_t cali;
    SENSOR_CALI_t levelCali;
} ACCELEROMETER_t;

void AccPreTreatInit(void);
void AccDataPreTreat(Vector3ForFloat accRaw, Vector3ForFloat* accData);
void AccCalibration(Vector3ForFloat accRaw);
void AccScaleCalibrate(Vector3ForFloat* acc);
void ImuLevelCalibration(void);

Vector3ForFloat GetAccOffsetCaliData(void);
Vector3ForFloat GetAccScaleCaliData(void);
Vector3ForFloat GetLevelCalibraData(void);

void AccCalibrateEnable(void);
void LevelCalibrateEnable(void);

float GetAccMag(void);
Vector3ForFloat AccGetData(void);
Vector3ForFloat AccLpfGetData(void);

#endif



