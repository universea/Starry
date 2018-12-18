#ifndef _LOWPASSFILTER_H_
#define _LOWPASSFILTER_H_

#include "mathTool.h"

typedef struct
{
    float b0;
    float a1;
    float a2;
    Vector3ForFloat preout;
    Vector3ForFloat lastout;
} LPF2ndData_t;

void LowPassFilter1st(Vector3ForFloat* data, Vector3ForFloat newData, float coff);
void LowPassFilter2ndFactorCal(float deltaT, float Fcut, LPF2ndData_t* lpf_data);
Vector3ForFloat LowPassFilter2nd(LPF2ndData_t* lpf_2nd, Vector3ForFloat rawData);

#endif


