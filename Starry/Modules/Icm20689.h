#ifndef __ICM20689_H
#define	__ICM20689_H

#ifdef __cplusplus
extern "C" {
#endif
	
#include "mathTool.h"

bool invSensorDetect(void);
void invSensorInit(void);
void invSensorRead(void);
	
void invSensorReadAccel(Vector3ForFloat* acc);
void invSensorReadGyro(Vector3ForFloat* gyro);
void invSensorReadTemperature(float* temp);

#ifdef __cplusplus
}
#endif
#endif








