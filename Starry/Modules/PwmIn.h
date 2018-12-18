#ifndef __PWMIN_H
#define __PWMIN_H
#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include "main.h"

enum
{
	CH1 = 0,
	CH2,
	CH3,
	CH4,
	CH5,
	CH6,
	CH7,
	CH8
};
	
void pwmInInit (void);
uint16_t getDutyFromPwmIn(uint8_t ch);

#ifdef __cplusplus
}
#endif
#endif
