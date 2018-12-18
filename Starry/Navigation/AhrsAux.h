#ifndef _AHRSAUX_H_
#define _AHRSAUX_H_

#include "mathTool.h"

typedef struct {
    Vector3ForFloat angle;
    float      q[4];
    Vector3ForFloat angleError;

    Vector3ForFloat vectorRollPitch;

    Vector3ForFloat accEf;
} AHRSAUX_t;

void AHRSAuxInit(void);
void AttitudeAuxEstimate(Vector3ForFloat gyro, Vector3ForFloat acc, Vector3ForFloat mag);

void RollPitchUpdateByKF(Vector3ForFloat* angle, Vector3ForFloat gyro, Vector3ForFloat acc, float deltaT);
void QuaternionUpdateByCF(float q[4], Vector3ForFloat* angle, Vector3ForFloat gyro, Vector3ForFloat acc, Vector3ForFloat mag, float deltaT);

Vector3ForFloat GetSportAccEf(void);
Vector3ForFloat GetAuxAngle(void);

#endif


