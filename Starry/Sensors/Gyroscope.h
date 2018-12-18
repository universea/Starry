#ifndef __GYROSCOPE_H
#define	__GYROSCOPE_H

#include "sensor.h"

typedef struct {
    Vector3ForFloat data;
    Vector3ForFloat dataLpf;
    float temperature;
    LPF2ndData_t lpf_2nd;
    SENSOR_CALI_t cali;
} GYROSCOPE_t;

void GyroPreTreatInit(void);
void GyroDataPreTreat(Vector3ForFloat gyroRaw, float temperature, Vector3ForFloat* gyroData, Vector3ForFloat* gyroLpfData);
void GyroCalibration(Vector3ForFloat gyroRaw);

uint8_t GetGyroCaliStatus(void);
void GyroCalibrateEnable(void);

Vector3ForFloat GyroGetData(void);
Vector3ForFloat GyroLpfGetData(void);
float GyroGetTemp(void);
Vector3ForFloat GetGyroOffsetCaliData(void);

#endif



