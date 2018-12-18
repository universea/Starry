/* ---------------------------------------------------------------------
 * 
 * navigation_task解析库函数:
 * 导航相关任务，包括姿态估计、速度估计和位置估计
 *
 *   修订操作      版本号       日期         修订人
 * ------------    ------    ----------    ------------
 *     创建        1.0       2018.01.19      Universea
 *
 * ---------------------------------------------------------------------*/
#include "TaskConfig.h"

#include "ahrs.h"
#include "ahrsAux.h"
#include "navigation.h"
#include "gyroscope.h"
#include "magnetometer.h"
#include "flightStatus.h"
#include "rc.h"

osThreadId navigationTask;
osThreadId flightStatusTask;

/**********************************************************************************************************
*函 数 名: vNavigationTask
*功能说明: 导航计算相关任务
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void startNavigationTask(void const * argument)
{
    Vector3ForFloat* gyro;
    Vector3ForFloat* acc;

    osDelay(500);

    //姿态估计参数初始化
    AHRSInit();
    //辅助姿态估计参数初始化
    AHRSAuxInit();
    //导航参数初始化
    NavigationInit();

    for(;;)
    {
        //从消息队列中获取数据
        xQueueReceive(messageQueue[GYRO_DATA_PRETREAT], &gyro, (3 / portTICK_RATE_MS));
        xQueueReceive(messageQueue[ACC_DATA_PRETREAT], &acc, (3 / portTICK_RATE_MS));

        //辅助姿态估计
        AttitudeAuxEstimate(*gyro, *acc, MagGetData());

        //姿态卡尔曼测量噪声协方差参数自适应
        AttCovarianceSelfAdaptation();

        //姿态估计
        AttitudeEstimate(*gyro, *acc, MagGetData());

        //等待系统初始化完成
        if(GetInitStatus() == INIT_FINISH)
        {
            //高度卡尔曼测量噪声协方差参数自适应
            AltCovarianceSelfAdaptation();

            //位置卡尔曼测量噪声协方差参数自适应
            PosCovarianceSelfAdaptation();

            //飞行速度估计
            VelocityEstimate();

            //位置估计
            PositionEstimate();
        }
				else
				{
						NavigationReset();
				}
    }
}

/**********************************************************************************************************
*函 数 名: vFlightStatusTask
*功能说明: 飞行状态检测相关任务
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void startFlightStatusTask(void const * argument)
{
    portTickType xLastWakeTime;

    xLastWakeTime = osKernelSysTick();

    for(;;)
    {
        //系统初始化检测
        SystemInitCheck();

        //飞行器放置状态检测
        PlaceStausCheck(GyroLpfGetData());

        //飞行状态更新
        FlightStatusUpdate();

        //环境风速估计
        WindEstimate();

        //传感器方向检测（用于校准时的判断）
        ImuOrientationDetect();

        //睡眠10ms
        osDelayUntil(&xLastWakeTime, (10 / portTICK_RATE_MS));
    }
}

/**********************************************************************************************************
*函 数 名: NavigationTaskCreate
*功能说明: 导航相关任务创建
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void NavigationTaskCreate(void)
{
		osThreadDef(navigation, startNavigationTask, NAVIGATION_TASK_PRIORITY, 0, NAVIGATION_TASK_STACK);
		navigationTask = osThreadCreate(osThread(navigation), NULL);
		osThreadDef(flightStatus, startFlightStatusTask, FLIGHT_STATUS_TASK_PRIORITY, 0, FLIGHT_STATUS_TASK_STACK);
		flightStatusTask = osThreadCreate(osThread(flightStatus), NULL);
}

/**********************************************************************************************************
*函 数 名: GetNavigationTaskStackRemain
*功能说明: 获取任务堆栈使用剩余
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
int16_t	GetNavigationTaskStackRemain(void)
{
    return uxTaskGetStackHighWaterMark(navigationTask);
}

/**********************************************************************************************************
*函 数 名: GetFlightStatusTaskStackRemain
*功能说明: 获取任务堆栈使用剩余
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
int16_t	GetFlightStatusTaskStackRemain(void)
{
    return uxTaskGetStackHighWaterMark(flightStatusTask);
}


