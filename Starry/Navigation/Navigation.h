#ifndef _NAVIGATION_H_
#define _NAVIGATION_H_

#include "mathTool.h"

typedef struct {
    Vector3ForFloat accel;
	  Vector3ForFloat accel_bias;
	
    Vector3ForFloat velocity;
    Vector3ForFloat velMeasure;
	
    Vector3ForFloat position;
    Vector3ForFloat posMeasure;
} NAVGATION_t;

void NavigationInit(void);
void VelocityEstimate(void);
void PositionEstimate(void);

void AltCovarianceSelfAdaptation(void);
void PosCovarianceSelfAdaptation(void);

Vector3ForFloat GetCopterAccel(void);
Vector3ForFloat GetCopterVelocity(void);
Vector3ForFloat GetCopterVelMeasure(void);
Vector3ForFloat GetCopterPosition(void);
Vector3ForFloat GetCopterPosMeasure(void);

void NavigationReset(void);

#endif







