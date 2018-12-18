#ifndef __MODULE_H__
#define __MODULE_H__

#ifdef __cplusplus
extern "C" {
#endif
	
#include "mathTool.h"

void GyroSensorInit(void);
void MagSensorInit(void);
void BaroSensorInit(void);
void TofSensorInit(void);
void GPSModuleInit(void);

void GyroSensorRead(Vector3ForFloat* gyro);
void AccSensorRead(Vector3ForFloat* acc);
void TempSensorRead(float* temp);

void MagSensorUpdate(void);
void MagSensorRead(Vector3ForFloat* mag);
void BaroSensorRead(int32_t* baroAlt);
void BaroSensorUpdate(void);
void BaroTemperatureRead(float* temp);

void TempControlSet(int16_t value);

#ifdef __cplusplus
}
#endif
#endif










