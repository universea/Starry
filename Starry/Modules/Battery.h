#ifndef __BATTERY_H
#define	__BATTERY_H

#ifdef __cplusplus
extern "C" {
#endif
	
#include "mathTool.h"

enum
{
    BATTERY_NORMAL,
    BATTERY_LOW,
    BATTERY_CRITICAL_LOW
};

void BatteryVoltageUpdate(void);
void BatteryCurrentUpdate(void);

int16_t GetBatteryVoltage(void);
int16_t GetBatteryCurrent(void);
uint8_t GetBatteryStatus(void);

#ifdef __cplusplus
}
#endif
#endif






