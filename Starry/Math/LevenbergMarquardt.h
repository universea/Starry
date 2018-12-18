#ifndef _LEVENBERGMARQUARDT_H
#define _LEVENBERGMARQUARDT_H

#include "mathTool.h"

void LevenbergMarquardt(Vector3ForFloat inputData[6], Vector3ForFloat* offset, Vector3ForFloat* scale, float initBeta[6], float length);

#endif
