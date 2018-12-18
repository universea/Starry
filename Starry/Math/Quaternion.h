#ifndef __QUATERNION_H__
#define __QUATERNION_H__

#include "mathTool.h"

void EulerAngleToQuaternion(Vector3ForFloat angle, float q[4]);
void QuaternionToDCM(float q[4], float dcM[9]);
void QuaternionToDCM_T(float q[4], float dcM[9]);
Vector3ForFloat QuaternionRotateToEarthFrame(float q[4], Vector3ForFloat vector);
Vector3ForFloat QuaternionRotateToBodyFrame(float q[4], Vector3ForFloat vector);
void QuaternionToEulerAngle(float q[4], Vector3ForFloat* angle);
void QuaternionNormalize(float q[4]);

#endif











