/* ---------------------------------------------------------------------
 * 
 * module_task解析库函数:
 * 传感器及外设等相关任务
 *
 *   修订操作      版本号       日期         修订人
 * ------------    ------    ----------    ------------
 *     创建        1.0       2018.01.19      Universea
 *
 * ---------------------------------------------------------------------*/
#include "TaskConfig.h"
#include "Ublox.h"
#include "module.h"
#include "battery.h"
#include "LedColors.h"
#include "parameter.h"
#include "flightStatus.h"

//声明任务句柄
osThreadId imuSensorReadTask;
osThreadId sensorUpdateTask;

/**********************************************************************************************************
*函 数 名: startImuSensorReadTask
*功能说明: IMU传感器数据读取任务，此任务具有最高优先级，运行频率为1KHz
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void startImuSensorReadTask(void const * argument)
{
    portTickType xLastWakeTime;

    Vector3ForFloat* accRawData  = pvPortMalloc(sizeof(Vector3ForFloat));
    Vector3ForFloat* gyroRawData = pvPortMalloc(sizeof(Vector3ForFloat));
    float*      tempRawData = pvPortMalloc(sizeof(float));

    //挂起调度器
    osThreadSuspendAll();

    //陀螺仪传感器初始化
    GyroSensorInit();

    //唤醒调度器
    osThreadResumeAll();

    xLastWakeTime = osKernelSysTick();
    for(;;)
    {
        //读取加速度传感器
        AccSensorRead(accRawData);
        //读取陀螺仪传感器
        GyroSensorRead(gyroRawData);
        //读取温度传感器
        TempSensorRead(tempRawData);

        //更新消息队列，通知数据预处理任务对IMU数据进行预处理
        xQueueSendToBack(messageQueue[ACC_SENSOR_READ],  (void *)&accRawData, 0);
        xQueueSendToBack(messageQueue[GYRO_SENSOR_READ],  (void *)&gyroRawData, 0);
        xQueueSendToBack(messageQueue[TEMP_SENSOR_READ],  (void *)&tempRawData, 0);
				
        //睡眠1ms
        osDelayUntil(&xLastWakeTime, (1 / portTICK_RATE_MS));
    }
}

/**********************************************************************************************************
*函 数 名: startSensorUpdateTask
*功能说明: IMU之外的传感器数据更新任务
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void startSensorUpdateTask(void const * argument)
{
    portTickType xLastWakeTime;
    static uint16_t count = 0;

    //挂起调度器
    osThreadSuspendAll();

    //地磁传感器初始化
    MagSensorInit();
    //气压传感器初始化
    BaroSensorInit();
		//TOF传感器初始化
		TofSensorInit();

    //唤醒调度器
    osThreadResumeAll();

    //GPS模块初始化
    GPSModuleInit();

    xLastWakeTime = osKernelSysTick();
    for(;;)
    {
        //地磁传感器数据更新 100Hz
        if(count % 2 == 0)
        {
					  osThreadSuspendAll();
            MagSensorUpdate();
						osThreadResumeAll();
        }

        //气压传感器数据更新 50Hz
        if(count % 4 == 0)
        {
            //读取气压计数据时挂起调度器，防止SPI总线冲突
            osThreadSuspendAll();
            BaroSensorUpdate();
            osThreadResumeAll();
        }

        //飞控参数保存(参数有更新才会执行）20Hz
        if(count % 10 == 0)
        {
            ParamSaveToFlash();
        }

        //电池电压电流采样更新 200Hz
        BatteryVoltageUpdate();
        BatteryCurrentUpdate();
				
				//RGB闪烁
        RGB_Flash();

        count++;

        //睡眠5ms
        osDelayUntil(&xLastWakeTime, (5 / portTICK_RATE_MS));
    }
}

/**********************************************************************************************************
*函 数 名: ModuleTaskCreate
*功能说明: 传感器组件相关任务创建
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void ModuleTaskCreate(void)
{
	  osThreadDef(imuSensorRead, startImuSensorReadTask, IMU_SENSOR_READ_TASK_PRIORITY, 0, IMU_SENSOR_READ_TASK_STACK);
		imuSensorReadTask = osThreadCreate(osThread(imuSensorRead), NULL);
		osThreadDef(sensorUpdate, startSensorUpdateTask, SENSOR_UPDATE_TASK_PRIORITY, 0, SENSOR_UPDATE_TASK_STACK);
		imuSensorReadTask = osThreadCreate(osThread(sensorUpdate), NULL);
}


/**********************************************************************************************************
*函 数 名: GetImuSensorReadTaskStackRemain
*功能说明: 获取任务堆栈使用剩余
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
int16_t	GetImuSensorReadTaskStackRemain(void)
{
    return uxTaskGetStackHighWaterMark(imuSensorReadTask);
}

/**********************************************************************************************************
*函 数 名: GetSensorUpdateTaskStackRemain
*功能说明: 获取任务堆栈使用剩余
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
int16_t	GetSensorUpdateTaskStackRemain(void)
{
    return uxTaskGetStackHighWaterMark(sensorUpdateTask);
}




