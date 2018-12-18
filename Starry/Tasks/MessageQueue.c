/* ---------------------------------------------------------------------
 * 
 * messageQueue解析库函数:
 * 消息队列，主要用于对数据传递实时性要求较高的任务间通信
 *
 *   修订操作      版本号       日期         修订人
 * ------------    ------    ----------    ------------
 *     创建        1.0       2018.01.19      Universea
 *
 * ---------------------------------------------------------------------*/
#include "messageQueue.h"

//声明消息队列句柄
osMessageQId messageQueue[QUEUE_NUM];

/**********************************************************************************************************
*函 数 名: MessageQueueCreate
*功能说明: 消息队列创建
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void MessageQueueCreate(void)
{
	  osMessageQDef(QueueOfAccelSensorRead, 2, sizeof(Vector3ForFloat *));
		messageQueue[ACC_SENSOR_READ] = osMessageCreate(osMessageQ(QueueOfAccelSensorRead), NULL);
		osMessageQDef(QueueOfGyroSensorRead, 2, sizeof(Vector3ForFloat *));
		messageQueue[GYRO_SENSOR_READ] = osMessageCreate(osMessageQ(QueueOfGyroSensorRead), NULL);
		osMessageQDef(QueueOfTemperatureSensorRead, 2, sizeof(float *));
		messageQueue[TEMP_SENSOR_READ] = osMessageCreate(osMessageQ(QueueOfTemperatureSensorRead), NULL);
	
		osMessageQDef(QueueOfGyroDataPretreat, 2, sizeof(Vector3ForFloat *));
		messageQueue[GYRO_DATA_PRETREAT] = osMessageCreate(osMessageQ(QueueOfGyroDataPretreat), NULL);
		osMessageQDef(QueueOfAccelDataPretreat, 2, sizeof(Vector3ForFloat *));
		messageQueue[ACC_DATA_PRETREAT] = osMessageCreate(osMessageQ(QueueOfAccelDataPretreat), NULL);
		osMessageQDef(QueueOfGyroForControl, 2, sizeof(Vector3ForFloat *));
		messageQueue[GYRO_FOR_CONTROL] = osMessageCreate(osMessageQ(QueueOfGyroForControl), NULL);
}






