/* ---------------------------------------------------------------------
 * 
 * parameter解析库函数:
 * 飞控参数
 *
 *   修订操作      版本号       日期         修订人
 * ------------    ------    ----------    ------------
 *     创建        1.0       2018.01.19      Universea
 *
 * ---------------------------------------------------------------------*/
#include "parameter.h"
#include "W25QXX.h"
#include "mathTool.h"
#include "flightStatus.h"


union Parameter myParameters;
parameterStateStruct parameterStates;

static void ParamReadFromFlash(void);


/**********************************************************************************************************
*函 数 名: ParamInit
*功能说明: 参数初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void ParamInit(void)
{
    ParamReadFromFlash();
}

/**********************************************************************************************************
*函 数 名: ParamDataReset
*功能说明: 参数置零
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
static void ParamDataReset(void)
{
		for(uint8_t i = 0;i<3;i++)
		{
			myParameters.set.GyroOffset[i] = 0;
			myParameters.set.GyroScale[i] = 0;
			myParameters.set.AccelOffset[i] = 0;
			myParameters.set.AccelScale[i] = 0;
			myParameters.set.MagOffset[i] = 0;
			myParameters.set.MagScale[i] = 0;
		}
}

/**********************************************************************************************************
*函 数 名: ParamReadFromFlash
*功能说明: 把飞控参数存储区的内容读取出来
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
static void ParamReadFromFlash(void)
{
    Flash_SectorsRead ( 0x000000, &myParameters.byte[0], 1 );		//读取第一扇区内的参数
	
		if(myParameters.set.FirstInit != 0x01)	//内容没有被初始化，则进行参数初始化工作
		{		
			ParamDataReset();
			//pidReset();
			//myParameWrite();
		}
}

/**********************************************************************************************************
*函 数 名: ParamSaveToFlash
*功能说明: 把飞控参数写入存储区 运行频率20Hz
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void ParamSaveToFlash(void)
{
//		//解锁后不得保存参数
    if(GetArmedStatus() == ARMED)
    {
        parameterStates.SaveTrig = 0;
        return;
    }
    //allPidInit();	//////存储PID参数后，重新初始化PID	
		if(parameterStates.TimeDelay == 2)
		{
				myParameters.set.FirstInit = 0x01;
				Flash_SectorErase ( 0x000000, 1 );							//擦除第一扇区
				Flash_SectorsWrite ( 0x000000, &myParameters.byte[0], 1 );	//将参数写入第一扇区
		}
		if(parameterStates.TimeDelay > 1)
		{
			parameterStates.TimeDelay--;
		}
		else
		{
			parameterStates.TimeDelay = 0;
		}
}

/**********************************************************************************************************
*函 数 名: ParamUpdateData
*功能说明: 更新飞控参数
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void ParamUpdateData(uint8_t saveTrig)
{
		parameterStates.SaveTrig = saveTrig;
    //参数更新的3秒后刷新一次Flash
    parameterStates.TimeDelay = 20*saveTrig;
}

