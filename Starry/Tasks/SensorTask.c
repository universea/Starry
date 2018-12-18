/* ---------------------------------------------------------------------
 * 
 * sensor_task解析库函数:
 * 传感器校准及数据预处理相关任务
 *
 *   修订操作      版本号       日期         修订人
 * ------------    ------    ----------    ------------
 *     创建        1.0       2018.01.19      Universea
 *
 * ---------------------------------------------------------------------*/
#include "TaskConfig.h"

#include "gyroscope.h"
#include "accelerometer.h"
#include "magnetometer.h"
#include "barometer.h"
#include "Tof.h"
#include "gps.h"

osThreadId imuDataPreTreatTask;
osThreadId otherSensorTask;

/**********************************************************************************************************
*函 数 名: vImuDataPreTreatTask
*功能说明: IMU传感器数据预处理任务，任务优先级仅次于IMU传感器读取
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void startImuDataPreTreatTask(void const * argument)
{
    Vector3ForFloat* gyroRawData;
    Vector3ForFloat* accRawData;
    float*      tempRawData;
    Vector3ForFloat* accData  = pvPortMalloc(sizeof(Vector3ForFloat));
    Vector3ForFloat* gyroData = pvPortMalloc(sizeof(Vector3ForFloat));
    Vector3ForFloat* gyroLpfData = pvPortMalloc(sizeof(Vector3ForFloat));

    //挂起调度器
    osThreadSuspendAll();

    //陀螺仪预处理初始化
    GyroPreTreatInit();
    //加速度预处理初始化
    AccPreTreatInit();
    //IMU传感器恒温参数初始化
    ImuTempControlInit();

    //唤醒调度器
    osThreadResumeAll();

    for(;;)
    {
        //从消息队列中获取数据
        xQueueReceive(messageQueue[GYRO_SENSOR_READ], &gyroRawData, (3 / portTICK_RATE_MS));
        xQueueReceive(messageQueue[ACC_SENSOR_READ], &accRawData, (3 / portTICK_RATE_MS));
        xQueueReceive(messageQueue[TEMP_SENSOR_READ], &tempRawData, (3 / portTICK_RATE_MS));

        //陀螺仪校准
        GyroCalibration(*gyroRawData);
        //加速度校准
        AccCalibration(*accRawData);

        //陀螺仪数据预处理
        GyroDataPreTreat(*gyroRawData, *tempRawData, gyroData, gyroLpfData);
        //加速度数据预处理
        AccDataPreTreat(*accRawData, accData);

        //IMU安装误差校准
        ImuLevelCalibration();

        //IMU传感器恒温控制
        ImuTempControl(*tempRawData);

        //往下一级消息队列中填充数据
        xQueueSendToBack(messageQueue[ACC_DATA_PRETREAT], (void *)&accData, 0);
        xQueueSendToBack(messageQueue[GYRO_DATA_PRETREAT], (void *)&gyroData, 0);
        xQueueSendToBack(messageQueue[GYRO_FOR_CONTROL], (void *)&gyroLpfData, 0);
    }
}

/**********************************************************************************************************
*函 数 名: vOtherSensorTask
*功能说明: 其它传感器数据预处理任务
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void startOtherSensorTask(void const * argument)
{
    portTickType xLastWakeTime;
    static uint16_t count = 0;

    //挂起调度器
    osThreadSuspendAll();

    //磁力计校准参数初始化
    MagCaliDataInit();

    //唤醒调度器
    osThreadResumeAll();

    xLastWakeTime = osKernelSysTick();
    for(;;)
    {
        //100Hz
        if(count % 2 == 0)
        {
            //磁力计校准
            MagCalibration();

            //磁力计数据预处理
            MagDataPreTreat();
        }

        //25Hz
        if(count % 8 == 0)
        {
            //气压高度数据预处理
            BaroDataPreTreat();
						TofDataPreTreat();
        }
				

        //10Hz
        if(count % 20 == 0)
        {
            //GPS数据预处理
            GpsDataPreTreat();
        }

        //传感器健康状态检测
        SensorHealthCheck();

        count++;

        //睡眠5ms
        osDelayUntil(&xLastWakeTime, (5 / portTICK_RATE_MS));
    }
}

/**********************************************************************************************************
*函 数 名: SensorTaskCreate
*功能说明: 传感器数据预处理相关任务创建
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void SensorTaskCreate(void)
{
		osThreadDef(imuDataPreTreat, startImuDataPreTreatTask, IMU_DATA_PRETREAT_TASK_PRIORITY, 0, IMU_DATA_PRETREAT_TASK_STACK);
		imuDataPreTreatTask = osThreadCreate(osThread(imuDataPreTreat), NULL);
		osThreadDef(otherSensor, startOtherSensorTask, OTHER_SENSOR_TASK_PRIORITY, 0, OTHER_SENSOR_TASK_STACK);
		otherSensorTask = osThreadCreate(osThread(otherSensor), NULL);
}

/**********************************************************************************************************
*函 数 名: GetImuDataPreTreatTaskStackRemain
*功能说明: 获取任务堆栈使用剩余
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
int16_t	GetImuDataPreTreatTaskStackRemain(void)
{
    return uxTaskGetStackHighWaterMark(imuDataPreTreatTask);
}

/**********************************************************************************************************
*函 数 名: GetOtherSensorTaskStackRemain
*功能说明: 获取任务堆栈使用剩余
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
int16_t	GetOtherSensorTaskStackRemain(void)
{
    return uxTaskGetStackHighWaterMark(otherSensorTask);
}

