#ifndef __AK8975_H
#define __AK8975_H
#ifdef __cplusplus
extern "C" {
#endif

#include "mathTool.h"
#include "stm32f4xx_hal.h"

bool ak8975Detect(void);
void ak8975Init(void);
void ak8975Update(void);
void ak8975Read(Vector3ForFloat* mag);
	
#ifdef __cplusplus
}
#endif
#endif
