#ifndef _AHRS_H_
#define _AHRS_H_

#include "mathTool.h"

typedef struct {
    Vector3ForFloat angle;
    Vector3ForFloat angleError;
    Vector3ForFloat angleMeasure;
    
    Vector3ForFloat accEf;
    Vector3ForFloat accEfLpf;
    Vector3ForFloat accBfOffset;

    Vector3ForFloat gyroEf;

    Vector3ForFloat centripetalAcc;
    Vector3ForFloat centripetalAccBf;
} AHRS_t;

void AHRSInit(void);
void AttitudeEstimate(Vector3ForFloat gyro, Vector3ForFloat acc, Vector3ForFloat mag);
void BodyFrameToEarthFrame(Vector3ForFloat angle, Vector3ForFloat vector, Vector3ForFloat* vectorEf);
void EarthFrameToBodyFrame(Vector3ForFloat angle, Vector3ForFloat vector, Vector3ForFloat* vectorBf);

void AttCovarianceSelfAdaptation(void);

Vector3ForFloat GetCopterAngle(void);
Vector3ForFloat GetCopterAccEf(void);
Vector3ForFloat GetCopterAccEfLpf(void);
Vector3ForFloat GetCentripetalAcc(void);
Vector3ForFloat GetCentripetalAccBf(void);
Vector3ForFloat GetAngleMeasure(void);
Vector3ForFloat GetAngleEstError(void);

uint8_t RollOverDetect(void);

#endif






