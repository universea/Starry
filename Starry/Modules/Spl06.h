#ifndef __SPL06_H
#define __SPL06_H
#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include "mathTool.h"

bool spl0601Detect(void);
void spl0601Init (void);
void spl0601Update(void);
void spl0601Read (int32_t* baroAlt);
void spl0601ReadTemperature(float* temp);
	
#ifdef __cplusplus
}
#endif
#endif
