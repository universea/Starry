#include "Tof.h"
#include "Vl53l0X.h"
#include "board.h"

typedef struct {
    int32_t alt;
    int32_t lastAlt;
    float   velocity;
    int32_t alt_offset;
    float temperature;
} TOF_t;

TOF_t tof;

/**********************************************************************************************************
*函 数 名: BaroDataPreTreat
*功能说明: 气压高度数据预处理
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void TofDataPreTreat(void)
{
    static uint64_t lastTime = 0;
    uint16_t tofAltTemp;
		float tofAlt;

    float deltaT = (getSysTimeUs() - lastTime) * 1e-6;
    lastTime = getSysTimeUs();

    //读取TOF高度
		vl53l0xRead(&tofAltTemp);
		tofAlt = 0.1f * tofAltTemp;

    //气压高度低通滤波
    tof.alt = tof.alt * 0.5f + tofAlt * 0.5f;

    //计算TOF变化速度，并进行低通滤波
    tof.velocity = tof.velocity * 0.65f + ((tof.alt - tof.lastAlt) / deltaT) * 0.35f;
    tof.lastAlt = tof.alt;
}

/**********************************************************************************************************
*函 数 名: TofGetAlt
*功能说明: 获取TOF高度数据
*形    参: 无
*返 回 值: 气压高度
**********************************************************************************************************/
int32_t TofGetAlt(void)
{
    return tof.alt;
}

/**********************************************************************************************************
*函 数 名: TofGetVelocity
*功能说明: 获取TOF高度变化速度
*形    参: 无
*返 回 值: 气压高度变化速度
**********************************************************************************************************/
float TofGetVelocity(void)
{
    return tof.velocity;
}