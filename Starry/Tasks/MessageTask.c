/* ---------------------------------------------------------------------
 * 
 * message_task解析库函数:
 * 飞控数据通信相关任务
 *
 *   修订操作      版本号       日期         修订人
 * ------------    ------    ----------    ------------
 *     创建        1.0       2018.01.19      Universea
 *
 * ---------------------------------------------------------------------*/
#include "TaskConfig.h"
#include "string.h"
#include "message.h"
#include "bsklinkDecode.h"
#include "mavlinkDecode.h"
osThreadId messageTask;

/**********************************************************************************************************
*函 数 名: vMessageTask
*功能说明: 数据通信任务
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void startMessageTask(void const * argument)
{
    portTickType xLastWakeTime;

    //飞控数据通信初始化
    MessageInit();

    xLastWakeTime = osKernelSysTick();
    for(;;)
    {
				if(decodeMode == 0)
				{
					  for(uint8_t i = 0; i<(*(messageTorReadBuffer)); i++)
						{
								MessageProtocolTypeDetect(*(messageTorReadBuffer+i));
						}
						memset(messageTorReadBuffer,0,sizeof(messageTorReadBuffer));//清除缓存
				}
				else if(decodeMode == 1)
				{
						for(uint8_t i = 0; i<(*(messageTorReadBuffer)); i++)
						{
								BsklinkDecode(*(messageTorReadBuffer+i));
						}
						memset(messageTorReadBuffer,0,sizeof(messageTorReadBuffer));//清除缓存
				}	
				else if(decodeMode == 2)
				{
						for(uint8_t i = 0; i<(*(messageTorReadBuffer)); i++)
						{
								MavlinkDecode(*(messageTorReadBuffer+i));
						}
						memset(messageTorReadBuffer,0,sizeof(messageTorReadBuffer));//清除缓存
				}
        //发送飞控数据
        MessageSendLoop();

        osDelayUntil(&xLastWakeTime, ((1000 / MAX_SEND_FREQ) / portTICK_RATE_MS));
    }
}

/**********************************************************************************************************
*函 数 名: MessageTaskCreate
*功能说明: 数据通信任务创建
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void MessageTaskCreate(void)
{
		osThreadDef(message, startMessageTask, MESSAGE_TASK_PRIORITY, 0, MESSAGE_TASK_STACK);
		messageTask = osThreadCreate(osThread(message), NULL);
}

/**********************************************************************************************************
*函 数 名: GetMessageTaskStackRemain
*功能说明: 获取任务堆栈使用剩余
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
int16_t	GetMessageTaskStackRemain(void)
{
    return uxTaskGetStackHighWaterMark(messageTask);
}


