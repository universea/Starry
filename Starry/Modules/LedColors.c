#include "LedColors.h"
#include "board.h"
#include "flightStatus.h"
#include "sensor.h"

#define LED_BLUE_OFF         HAL_GPIO_WritePin(GPIOE, SKY_LED_0_Pin, GPIO_PIN_SET);
#define LED_BLUE_ON          HAL_GPIO_WritePin(GPIOE, SKY_LED_0_Pin, GPIO_PIN_RESET);
#define LED_GREEN_ON          HAL_GPIO_WritePin(GPIOE, SKY_LED_1_Pin, GPIO_PIN_SET);
#define LED_GREEN_OFF         HAL_GPIO_WritePin(GPIOE, SKY_LED_1_Pin, GPIO_PIN_RESET);
#define LED_RED_ON          HAL_GPIO_WritePin(GPIOE, SKY_LED_2_Pin, GPIO_PIN_SET);
#define LED_RED_OFF         HAL_GPIO_WritePin(GPIOE, SKY_LED_2_Pin, GPIO_PIN_RESET);
#define LED4_ON          HAL_GPIO_WritePin(GPIOE, SKY_LED_3_Pin, GPIO_PIN_SET);
#define LED4_OFF         HAL_GPIO_WritePin(GPIOE, SKY_LED_3_Pin, GPIO_PIN_RESET);

void RGB_Red_On(void)
{
    HAL_GPIO_WritePin(GPIOE, SKY_LED_1_Pin, GPIO_PIN_RESET);
}

void RGB_Red_Off(void)
{
    HAL_GPIO_WritePin(GPIOE, SKY_LED_1_Pin, GPIO_PIN_SET);
}

void RGB_Green_On(void)
{
    HAL_GPIO_WritePin(GPIOE, SKY_LED_2_Pin, GPIO_PIN_RESET);
}

void RGB_Green_Off(void)
{
    HAL_GPIO_WritePin(GPIOE, SKY_LED_2_Pin, GPIO_PIN_SET);
}

void RGB_Blue_On(void)
{
    HAL_GPIO_WritePin(GPIOE, SKY_LED_3_Pin, GPIO_PIN_RESET);
}

void RGB_Blue_Off(void)
{
    HAL_GPIO_WritePin(GPIOE, SKY_LED_3_Pin, GPIO_PIN_SET);
}

/**********************************************************************************************************
*函 数 名: RGB_Init
*功能说明: RGB初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void RGB_Init(void)
{
    RGB_Green_Off();
    RGB_Red_Off();
    RGB_Blue_Off();
}

/**********************************************************************************************************
*函 数 名: RGB_Flash
*功能说明: RGB闪烁 运行频率200Hz
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void RGB_Flash(void)
{
    static uint32_t cnt = 0;
    static uint8_t initFlag = 0;
    
    switch(GetInitStatus())
    {
    case HEATING:
        if(initFlag == 0)
        {
            if(cnt > 250)
            {
                initFlag = 1;
            }
            else if(cnt > 150)
            {
                RGB_Green_Off();
                RGB_Red_Off();
                RGB_Blue_Off(); 
            }
            else if(cnt > 100)
            {
                RGB_Green_On();
                RGB_Red_Off();
                RGB_Blue_Off(); 
            }
            else if(cnt > 50)
            {
                RGB_Green_Off();
                RGB_Red_On();
                RGB_Blue_Off(); 
            }
            else
            {
                RGB_Green_Off();
                RGB_Red_Off();
                RGB_Blue_On();
            }
        }
        else
        { 
            //传感器检测正常则红灯慢闪，不正常快闪
            if(SensorCheckStatus())
            {
                if(cnt % 100 == 0)
                {
                    RGB_Green_Off();
                    RGB_Red_On();
                    RGB_Blue_Off(); 
                }
                if(cnt % 200 == 0)
                {
                    RGB_Green_Off();
                    RGB_Red_Off();
                    RGB_Blue_Off(); 
                }
            }
            else
            {
                if(cnt % 10 == 0)
                {
                    RGB_Green_Off();
                    RGB_Red_On();
                    RGB_Blue_Off(); 
                }
                if(cnt % 20 == 0)
                {
                    RGB_Green_Off();
                    RGB_Red_Off();
                    RGB_Blue_Off(); 
                }
            }
        }
        break;

    case HEAT_FINISH:
            if(cnt % 100 == 0)
            {
                RGB_Green_Off();
                RGB_Red_Off();
                RGB_Blue_On(); 
            }
            if(cnt % 200 == 0)
            {
                RGB_Green_Off();
                RGB_Red_Off();
                RGB_Blue_Off(); 
            }
        break;

    case INIT_FINISH:
            if(cnt % 10 == 0)
            {
                RGB_Green_Off();
                RGB_Red_Off();
                RGB_Blue_Off(); 
            }
            if(cnt % 300 == 0)
            {
                RGB_Green_On();
                RGB_Red_Off();
                RGB_Blue_Off(); 
            }
        break;

    default:
        break;
    }
    
    cnt++;
}

/******************* (C) COPYRIGHT 2016 SKY TECH *****END OF FILE************/