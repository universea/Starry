/* ---------------------------------------------------------------------
 * 
 * motor库函数:
 * 电机动力分配
 *
 *   修订操作      版本号       日期         修订人
 * ------------    ------    ----------    ------------
 *     创建        1.0       2018.01.19      Universea
 *
 * ---------------------------------------------------------------------*/
#include "motor.h"
#include "flightStatus.h"
#include "message.h"
#include "sensor.h"
#include "parameter.h"

//油门行程为[0:2000]
#define MINTHROTTLE	    0                     //最小油门值           
#define MAXTHROTTLE 		2000                    //最大油门值

//机型选择
#define motorType quadX

static int16_t motorPWM[8];
uint8_t escCaliFlag = 0;

//四轴X型
const MOTOR_TYPE_t quadX =
{
    .motorNum   = 4,                            //电机数量
    .motorMixer =
    {
        { 1.0f, -1.0f,  1.0f, -1.0f },          //后右
        { 1.0f, -1.0f, -1.0f,  1.0f },          //前右
        { 1.0f,  1.0f,  1.0f,  1.0f },          //后左
        { 1.0f,  1.0f, -1.0f, -1.0f },          //前左
    }
};

//六轴X型
const MOTOR_TYPE_t hex6X =
{
    .motorNum   = 6,                            //电机数量
    .motorMixer =
    {
        { 1.0f, -0.5f,  0.866025f,  1.0f },     //后右
        { 1.0f, -0.5f, -0.866025f,  1.0f },     //前右
        { 1.0f,  0.5f,  0.866025f, -1.0f },     //后左
        { 1.0f,  0.5f, -0.866025f, -1.0f },     //前左
        { 1.0f, -1.0f,  0.0f,      -1.0f },     //右
        { 1.0f,  1.0f,  0.0f,       1.0f },     //左
    }
};

//八轴X型
const MOTOR_TYPE_t octoFlatX =
{
    .motorNum   = 8,                            //电机数量
    .motorMixer =
    {
        { 1.0f,  1.0f, -0.5f,  1.0f },          //中前左
        { 1.0f, -0.5f, -1.0f,  1.0f },          //前右
        { 1.0f, -1.0f,  0.5f,  1.0f },          //中后右
        { 1.0f,  0.5f,  1.0f,  1.0f },          //后左
        { 1.0f,  0.5f, -1.0f, -1.0f },          //前左
        { 1.0f, -1.0f, -0.5f, -1.0f },          //中前右
        { 1.0f, -0.5f,  1.0f, -1.0f },          //后右
        { 1.0f,  1.0f,  0.5f, -1.0f },          //中后左
    }
};

void motorPwnSet(uint8_t motor,uint16_t pwmValue);

/**********************************************************************************************************
*函 数 名: MotorInit
*功能说明: 电机控制初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void MotorInit(void)
{
		escCaliFlag = myParameters.set.EscCalibrationFlag;
}

/**********************************************************************************************************
*函 数 名: MotorControl
*功能说明: 电机控制
*形    参: 横滚控制量 俯仰控制量 偏航控制量 油门控制量
*返 回 值: 无
**********************************************************************************************************/
void MotorControl(int16_t roll, int16_t pitch, int16_t yaw, int16_t throttle)
{
    int16_t maxMotorValue;
    static int16_t motorResetPWM[8];

    //电机动力分配
    for(uint8_t i=0; i<motorType.motorNum; i++)
    {
        motorPWM[i] = throttle * motorType.motorMixer[i].throttle +
                      roll     * motorType.motorMixer[i].roll     +
                      pitch    * motorType.motorMixer[i].pitch    +
                      yaw      * motorType.motorMixer[i].yaw;
    }

    //防止电机输出饱和
    maxMotorValue = motorPWM[0];
    for (uint8_t i=1; i<motorType.motorNum; i++)
    {
        if(motorPWM[i] > maxMotorValue)
            maxMotorValue = motorPWM[i];
    }
    for (uint8_t i=0; i<motorType.motorNum; i++)
    {
        if (maxMotorValue > MAXTHROTTLE)
            motorPWM[i] -= maxMotorValue - MAXTHROTTLE;
        //限制电机输出的最大最小值
        motorPWM[i] = ConstrainInt16(motorPWM[i], MINTHROTTLE, MAXTHROTTLE);
    }

    //电调校准
    if(escCaliFlag)
    {
        if(getSysTimeMs() < 6000)
        {
            for (uint8_t i=0; i<motorType.motorNum; i++)
            {
                motorPWM[i] = 2000;
            }
        }
        else
        {
            escCaliFlag = 0;
            //保存电调校准标志
						myParameters.set.EscCalibrationFlag = escCaliFlag;
						ParamUpdateData(1);//写入
        }
    }

    //判断飞控锁定状态，并输出电机控制量
    if(GetArmedStatus() == ARMED || escCaliFlag)
    {
        for (uint8_t i=0; i<motorType.motorNum; i++)
        {
            //输出PWM
            motorPwnSet(i+1, motorPWM[i]);
            //记录当前PWM值
            motorResetPWM[i] = motorPWM[i];
        }
    }
    else
    {
        for (uint8_t i=0; i<motorType.motorNum; i++)
        {
            //电机逐步减速，防止电机刹车引发射桨
            motorResetPWM[i] -= motorResetPWM[i] * 0.003f;
            motorPwnSet(i+1, motorResetPWM[i]);
        }
    }
}

/**********************************************************************************************************
*函 数 名: EscCalibrateEnable
*功能说明: 电调油门行程校准使能
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void EscCalibrateEnable(void)
{
    uint8_t caliFlag = 1;
    
    //保存电调校准标志
    myParameters.set.EscCalibrationFlag = caliFlag;
		ParamUpdateData(1);
    
    //发送校准结果
    MessageSensorCaliFeedbackEnable(ESC, 0, 1);
}

/**********************************************************************************************************
*函 数 名: MotorStop
*功能说明: 所有电机停转
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void MotorStop(void)
{
    for (uint8_t i=0; i<motorType.motorNum; i++)
    {
        motorPwnSet(i+1, 0);
    }
}

/**********************************************************************************************************
*函 数 名: GetMotorValue
*功能说明: 获取电机PWM值
*形    参: 无
*返 回 值: PWM值
**********************************************************************************************************/
int16_t* GetMotorValue(void)
{
    return motorPWM;
}

/**********************************************************************************************************
*函 数 名: GetMotorNum
*功能说明: 获取电机数量
*形    参: 无
*返 回 值: PWM值
**********************************************************************************************************/
int8_t GetMotorNum(void)
{
    return motorType.motorNum;
}
/**********************************************************************************************************
*函 数 名: motorPwnSet
*功能说明: 电调ESCPWM给定
*形    参: 无
*返 回 值: PWM值
**********************************************************************************************************/
uint16_t motors[4] = {0,0,0,0};

void motorPwnSet(uint8_t motor,uint16_t pwmValue)
{
		if(motor == 1)
		{
			TIM1->CCR1 = (uint16_t)(1.0f * ( pwmValue )) + 4000;
		}
		if(motor == 2)
		{
			TIM1->CCR4 = (uint16_t)(1.0f * ( pwmValue )) + 4000;
		}	
		if(motor == 3)
		{
			TIM1->CCR2 = (uint16_t)(1.0f * ( pwmValue )) + 4000;
		}	
		if(motor == 4)
		{
			TIM1->CCR3 = (uint16_t)(1.0f * ( pwmValue )) + 4000;
		}				
}