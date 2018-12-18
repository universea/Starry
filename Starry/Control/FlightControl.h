#ifndef _FLIGHTCONTROL_H_
#define _FLIGHTCONTROL_H_

#include "mathTool.h"
#include "pid.h"

#define MAXANGLE  400               //最大飞行角度：40°
#define MAXRCDATA 450

enum
{
    ROLL_INNER,
    PITCH_INNER,
    YAW_INNER,
    ROLL_OUTER,
    PITCH_OUTER,
    YAW_OUTER,
    VEL_X,
    VEL_Y,
    VEL_Z,
    POS_X,
    POS_Y,
    POS_Z,
    PIDNUM
};

typedef struct
{
    float roll;
    float pitch;
    float yaw;
    float throttle;
} RCTARGET_t;

typedef struct
{
    PID_t      pid[PIDNUM];         //PID参数结构体

    RCTARGET_t rcTarget;            //摇杆控制量
    Vector3ForFloat angleLpf;

    Vector3ForFloat attInnerCtlValue;    //姿态内环控制量
    float      altInnerCtlValue;    //高度内环控制量

    Vector3ForFloat attInnerTarget;      //姿态内环（角速度）控制目标
    Vector3ForFloat attOuterTarget;      //姿态外环（角度）控制目标
    Vector3ForFloat posInnerTarget;      //位置内环（速度）控制目标
    Vector3ForFloat posOuterTarget;      //位置外环（位置）控制目标

    Vector3ForFloat attInnerError;       //姿态内环（角速度）控制误差
    Vector3ForFloat attOuterError;       //姿态外环（角度）控制误差
    Vector3ForFloat posInnerError;       //位置内环（速度）控制误差
    Vector3ForFloat posOuterError;       //位置外环（位置）控制误差

    uint8_t    altCtlFlag;          //高度控制使能标志位
    uint8_t    posCtlFlag;          //位置控制使能标志位
    uint8_t    yawHoldFlag;         //航向锁定控制使能标志位

    int16_t    maxBrakeAngle;       //最大刹车角度
    int16_t    maxPosOuterCtl;      //位置控制的最大输出
    int16_t    maxAltOuterCtl;      //高度控制的最大输出
} FLIGHTCONTROL_t;

void FlightControlInit(void);

void SetRcTarget(RCTARGET_t rcTarget);
void FlightControlInnerLoop(Vector3ForFloat gyro);
void AttitudeOuterControl(void);
void AltitudeOuterControl(void);
void PositionInnerControl(void);
void PositionOuterControl(void);

void SetYawCtlTarget(float target);

void SetAltInnerCtlTarget(float target);
void SetAltOuterCtlTarget(float target);
void SetPosInnerCtlTarget(Vector3ForFloat target);
void SetPosOuterCtlTarget(Vector3ForFloat target);

void SetAltCtlStatus(uint8_t status);
void SetPosCtlStatus(uint8_t status);
void SetYawHoldStatus(uint8_t status);

Vector3ForFloat GetAttInnerCtlError(void);
Vector3ForFloat GetAttOuterCtlError(void);
Vector3ForFloat GetPosInnerCtlError(void);
Vector3ForFloat GetPosOuterCtlError(void);

Vector3ForFloat GetAttInnerCtlTarget(void);
Vector3ForFloat GetAttOuterCtlTarget(void);
Vector3ForFloat GetPosInnerCtlTarget(void);
Vector3ForFloat GetPosOuterCtlTarget(void);

void FlightControlReset(void);
PID_t FcGetPID(uint8_t id);
void FcSetPID(uint8_t id, PID_t pid);
bool PIDReadFromFlash(void);

void SetMaxBrakeAngle(int16_t angle);
void SetMaxPosOuterCtl(int16_t vel);
void SetMaxAltOuterCtl(int16_t vel);

#endif



