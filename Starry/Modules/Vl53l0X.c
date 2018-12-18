#include "Vl53l0X.h"
#include "vl53l0x_def.h"
#include "vl53l0x_platform.h"

uint8_t VL53L0X_LINKOK = 0;

/**********************************************************************************************************
*函 数 名: vl53l0xInit
*功能说明: tof 初始化函数
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void vl53l0xInit(void)
{
	if(VL53L0X_Init() == VL53L0X_ERROR_NONE)
	{
		VL53L0X_LINKOK = 1;
	}
	else
	{
		VL53L0X_LINKOK = 0;
	}
        if(VL53L0X_LINKOK==0)
        {
          	if(VL53L0X_Init() == VL53L0X_ERROR_NONE)
                {
                        VL53L0X_LINKOK = 1;
                }
                else
                {
                        VL53L0X_LINKOK = 0;
                }
        }
}

/**********************************************************************************************************
*函 数 名: vl53l0xRead
*功能说明: 读取tof数据
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void vl53l0xRead(uint16_t* tofHeightMm)
{
	if(!VL53L0X_LINKOK)
		return;
	
		*tofHeightMm = VL53L0X_FastRead();
}


