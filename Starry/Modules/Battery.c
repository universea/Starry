/* ---------------------------------------------------------------------
 * 
 * Battery库函数:
 * 电池操作函数.
 *
 *   修订操作      版本号       日期         修订人
 * ------------    ------    ----------    ------------
 *     创建        1.0       2018.01.19      Universea
 *
 * ---------------------------------------------------------------------*/
#include "Battery.h"
//#include "drv_adc.h"

#define VOLTAGE_LOW            360 * 4
#define VOLTAGE_CRITICAL_LOW   340 * 4

static float batVoltage;
static float batCurrent;

/**********************************************************************************************************
*函 数 名: BatteryVoltageUpdate
*功能说明: 电池电压采样更新
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void BatteryVoltageUpdate(void)
{
    if(batVoltage == 0)
    {
        //batVoltage = GetVoltageAdcValue() * ADC_VOLTAGE_COEF;
    }
    else
    {
        //batVoltage = batVoltage * 0.999f + GetVoltageAdcValue() * ADC_VOLTAGE_COEF * 0.001f;
    }
}

/**********************************************************************************************************
*函 数 名: BatteryCurrentUpdate
*功能说明: 电池电流采样更新
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void BatteryCurrentUpdate(void)
{
    if(batCurrent == 0)
    {
        //batCurrent = GetCurrentAdcValue() * ADC_CURRENT_COEF;
    }
    else
    {
        //batCurrent = batCurrent * 0.99f + GetCurrentAdcValue() * ADC_CURRENT_COEF * 0.01f;
    }
}

/**********************************************************************************************************
*函 数 名: GetBatteryVoltage
*功能说明: 获取电池电压
*形    参: 无
*返 回 值: 电压值
**********************************************************************************************************/
int16_t GetBatteryVoltage(void)
{
    return batVoltage;
}

/**********************************************************************************************************
*函 数 名: GetBatteryCurrent
*功能说明: 获取电池电流
*形    参: 无
*返 回 值: 电压值
**********************************************************************************************************/
int16_t GetBatteryCurrent(void)
{
    return batCurrent;
}

/**********************************************************************************************************
*函 数 名: GetBatteryStatus
*功能说明: 获取电池状态
*形    参: 无
*返 回 值: 状态
**********************************************************************************************************/
uint8_t GetBatteryStatus(void)
{
//    if(batVoltage < VOLTAGE_CRITICAL_LOW)
//    {
//        return BATTERY_CRITICAL_LOW;
//    }
//    else if(batVoltage < VOLTAGE_LOW)
//    {
//        return BATTERY_LOW;
//    }
//    else
//    {
//        return BATTERY_NORMAL;
//    }
		return BATTERY_NORMAL;
}
