#ifndef _VL53L0X_H_
#define _VL53L0X_H_
#include "stm32f4xx.h"


void vl53l0xInit(void);
void vl53l0xRead(uint16_t* tofHeightMm);

#endif
