#ifndef __TASKCONFIG_H__
#define __TASKCONFIG_H__

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "cmsis_os.h"

#include "board.h"
#include "mathTool.h"

#include "moduletask.h"
#include "sensortask.h"
#include "navigationtask.h"
#include "controltask.h"
#include "messagetask.h"
#include "logtask.h"

//任务堆栈大小
#define IMU_SENSOR_READ_TASK_STACK            256
#define SENSOR_UPDATE_TASK_STACK              256
#define IMU_DATA_PRETREAT_TASK_STACK          256
#define OTHER_SENSOR_TASK_STACK               256
#define NAVIGATION_TASK_STACK                 512
#define FLIGHT_STATUS_TASK_STACK              256
#define FLIGHTCONTROL_TASK_STACK              256
#define MESSAGE_TASK_STACK                    512
#define LOG_TASK_STACK                        1024

//任务优先级
//#define IMU_SENSOR_READ_TASK_PRIORITY         +10
//#define IMU_DATA_PRETREAT_TASK_PRIORITY       +9
//#define FLIGHTCONTROL_TASK_PRIORITY           +7
//#define NAVIGATION_TASK_PRIORITY              +8
//#define SENSOR_UPDATE_TASK_PRIORITY           +5
//#define OTHER_SENSOR_TASK_PRIORITY            +4
//#define MESSAGE_TASK_PRIORITY                 +3
//#define FLIGHT_STATUS_TASK_PRIORITY           +2
//#define LOG_TASK_PRIORITY                     +0
#define IMU_SENSOR_READ_TASK_PRIORITY         osPriorityRealtime
#define IMU_DATA_PRETREAT_TASK_PRIORITY       osPriorityHigh
#define FLIGHTCONTROL_TASK_PRIORITY           osPriorityAboveNormal
#define NAVIGATION_TASK_PRIORITY              osPriorityAboveNormal
#define SENSOR_UPDATE_TASK_PRIORITY           osPriorityNormal
#define OTHER_SENSOR_TASK_PRIORITY            osPriorityNormal
#define MESSAGE_TASK_PRIORITY                 osPriorityBelowNormal
#define FLIGHT_STATUS_TASK_PRIORITY           osPriorityBelowNormal
#define LOG_TASK_PRIORITY                     osPriorityIdle
/*
  osPriorityIdle          = -3,          ///< priority: idle (lowest)
  osPriorityLow           = -2,          ///< priority: low
  osPriorityBelowNormal   = -1,          ///< priority: below normal
  osPriorityNormal        =  0,          ///< priority: normal (default)
  osPriorityAboveNormal   = +1,          ///< priority: above normal
  osPriorityHigh          = +2,          ///< priority: high
  osPriorityRealtime      = +3,          ///< priority: realtime (highest)
*/

enum {
    GYRO_SENSOR_READ,
    ACC_SENSOR_READ,
    TEMP_SENSOR_READ,
    GYRO_DATA_PRETREAT,
    ACC_DATA_PRETREAT,
    GYRO_FOR_CONTROL,
    QUEUE_NUM
};

extern QueueHandle_t messageQueue[QUEUE_NUM];

#endif
