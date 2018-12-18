#ifndef __VECTOR3_H__
#define __VECTOR3_H__

#ifdef __cplusplus
extern "C" {
#endif
	
#include "mathTool.h"

typedef struct {
    float x;
    float y;
    float z;
} Vector3ForFloat;

typedef struct {
    double x;
    double y;
    double z;
} Vector3ForDouble;

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} Vector3ForInt16;

typedef struct {
    int32_t x;
    int32_t y;
    int32_t z;
} Vector3ForInt32;

void Vector3f_Normalize(Vector3ForFloat* vector);

Vector3ForFloat Vector3iTo3f(Vector3ForInt16 vector);
Vector3ForInt16 Vector3fTo3i(Vector3ForFloat vector);
Vector3ForFloat Vector3f_Add(Vector3ForFloat v1, Vector3ForFloat v2);
Vector3ForFloat Vector3f_Sub(Vector3ForFloat v1, Vector3ForFloat v2);

Vector3ForFloat VectorCrossProduct(Vector3ForFloat a, Vector3ForFloat b);
Vector3ForFloat Matrix3MulVector3(float* m, Vector3ForFloat vector);
Vector3ForFloat VectorRotateToBodyFrame(Vector3ForFloat vector, Vector3ForFloat deltaAngle);
Vector3ForFloat VectorRotateToEarthFrame(Vector3ForFloat vector, Vector3ForFloat deltaAngle);

void EulerAngleToDCM(Vector3ForFloat angle, float* dcM);

void AccVectorToRollPitchAngle(Vector3ForFloat* angle, Vector3ForFloat vector);
void MagVectorToYawAngle(Vector3ForFloat* angle, Vector3ForFloat vector);

#ifdef __cplusplus
}
#endif
#endif


