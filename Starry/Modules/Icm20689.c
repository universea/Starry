/* ---------------------------------------------------------------------
 * 
 * ICM20689库函数:
 * 加速度计和陀螺仪操作函数.
 *
 *   修订操作      版本号       日期         修订人
 * ------------    ------    ----------    ------------
 *     创建        1.0       2018.01.19      Universea
 *
 * ---------------------------------------------------------------------*/
#include "Icm20689.h"
#include "board.h"
#include "gpio.h"
#include "spi.h"

#define INVSENSOR_RA_XG_OFFS_TC       0x00    //[7] PWR_MODE, [6:1] XG_OFFS_TC, [0] OTP_BNK_VLD
#define INVSENSOR_RA_YG_OFFS_TC       0x01    //[7] PWR_MODE, [6:1] YG_OFFS_TC, [0] OTP_BNK_VLD
#define INVSENSOR_RA_ZG_OFFS_TC       0x02    //[7] PWR_MODE, [6:1] ZG_OFFS_TC, [0] OTP_BNK_VLD
#define INVSENSOR_RA_X_FINE_GAIN      0x03    //[7:0] X_FINE_GAIN
#define INVSENSOR_RA_Y_FINE_GAIN      0x04    //[7:0] Y_FINE_GAIN
#define INVSENSOR_RA_Z_FINE_GAIN      0x05    //[7:0] Z_FINE_GAIN
#define INVSENSOR_RA_XA_OFFS_H        0x06    //[15:0] XA_OFFS
#define INVSENSOR_RA_XA_OFFS_L_TC     0x07
#define INVSENSOR_RA_YA_OFFS_H        0x08    //[15:0] YA_OFFS
#define INVSENSOR_RA_YA_OFFS_L_TC     0x09
#define INVSENSOR_RA_ZA_OFFS_H        0x0A    //[15:0] ZA_OFFS
#define INVSENSOR_RA_ZA_OFFS_L_TC     0x0B
#define INVSENSOR_RA_PRODUCT_ID       0x0C    // Product ID Register
#define INVSENSOR_RA_XG_OFFS_USRH     0x13    //[15:0] XG_OFFS_USR
#define INVSENSOR_RA_XG_OFFS_USRL     0x14
#define INVSENSOR_RA_YG_OFFS_USRH     0x15    //[15:0] YG_OFFS_USR
#define INVSENSOR_RA_YG_OFFS_USRL     0x16
#define INVSENSOR_RA_ZG_OFFS_USRH     0x17    //[15:0] ZG_OFFS_USR
#define INVSENSOR_RA_ZG_OFFS_USRL     0x18
#define INVSENSOR_RA_SMPLRT_DIV       0x19
#define INVSENSOR_RA_CONFIG           0x1A
#define INVSENSOR_RA_GYRO_CONFIG      0x1B
#define INVSENSOR_RA_ACCEL_CONFIG     0x1C
#define INVSENSOR_RA_ACCEL_CONFIG2    0x1D
#define INVSENSOR_RA_FF_DUR           0x1E
#define INVSENSOR_RA_MOT_THR          0x1F
#define INVSENSOR_RA_MOT_DUR          0x20
#define INVSENSOR_RA_ZRMOT_THR        0x21
#define INVSENSOR_RA_ZRMOT_DUR        0x22
#define INVSENSOR_RA_FIFO_EN          0x23
#define INVSENSOR_RA_I2C_MST_CTRL     0x24
#define INVSENSOR_RA_I2C_SLV0_ADDR    0x25
#define INVSENSOR_RA_I2C_SLV0_REG     0x26
#define INVSENSOR_RA_I2C_SLV0_CTRL    0x27
#define INVSENSOR_RA_I2C_SLV1_ADDR    0x28
#define INVSENSOR_RA_I2C_SLV1_REG     0x29
#define INVSENSOR_RA_I2C_SLV1_CTRL    0x2A
#define INVSENSOR_RA_I2C_SLV2_ADDR    0x2B
#define INVSENSOR_RA_I2C_SLV2_REG     0x2C
#define INVSENSOR_RA_I2C_SLV2_CTRL    0x2D
#define INVSENSOR_RA_I2C_SLV3_ADDR    0x2E
#define INVSENSOR_RA_I2C_SLV3_REG     0x2F
#define INVSENSOR_RA_I2C_SLV3_CTRL    0x30
#define INVSENSOR_RA_I2C_SLV4_ADDR    0x31
#define INVSENSOR_RA_I2C_SLV4_REG     0x32
#define INVSENSOR_RA_I2C_SLV4_DO      0x33
#define INVSENSOR_RA_I2C_SLV4_CTRL    0x34
#define INVSENSOR_RA_I2C_SLV4_DI      0x35
#define INVSENSOR_RA_I2C_MST_STATUS   0x36
#define INVSENSOR_RA_INT_PIN_CFG      0x37
#define INVSENSOR_RA_INT_ENABLE       0x38
#define INVSENSOR_RA_DMP_INT_STATUS   0x39
#define INVSENSOR_RA_INT_STATUS       0x3A
#define INVSENSOR_RA_ACCEL_XOUT_H     0x3B
#define INVSENSOR_RA_ACCEL_XOUT_L     0x3C
#define INVSENSOR_RA_ACCEL_YOUT_H     0x3D
#define INVSENSOR_RA_ACCEL_YOUT_L     0x3E
#define INVSENSOR_RA_ACCEL_ZOUT_H     0x3F
#define INVSENSOR_RA_ACCEL_ZOUT_L     0x40
#define INVSENSOR_RA_TEMP_OUT_H       0x41
#define INVSENSOR_RA_TEMP_OUT_L       0x42
#define INVSENSOR_RA_GYRO_XOUT_H      0x43
#define INVSENSOR_RA_GYRO_XOUT_L      0x44
#define INVSENSOR_RA_GYRO_YOUT_H      0x45
#define INVSENSOR_RA_GYRO_YOUT_L      0x46
#define INVSENSOR_RA_GYRO_ZOUT_H      0x47
#define INVSENSOR_RA_GYRO_ZOUT_L      0x48
#define INVSENSOR_RA_EXT_SENS_DATA_00 0x49
#define INVSENSOR_RA_MOT_DETECT_STATUS    0x61
#define INVSENSOR_RA_I2C_SLV0_DO      0x63
#define INVSENSOR_RA_I2C_SLV1_DO      0x64
#define INVSENSOR_RA_I2C_SLV2_DO      0x65
#define INVSENSOR_RA_I2C_SLV3_DO      0x66
#define INVSENSOR_RA_I2C_MST_DELAY_CTRL   0x67
#define INVSENSOR_RA_SIGNAL_PATH_RESET    0x68
#define INVSENSOR_RA_MOT_DETECT_CTRL      0x69
#define INVSENSOR_RA_USER_CTRL        0x6A
#define INVSENSOR_RA_PWR_MGMT_1       0x6B
#define INVSENSOR_RA_PWR_MGMT_2       0x6C
#define INVSENSOR_RA_BANK_SEL         0x6D
#define INVSENSOR_RA_MEM_START_ADDR   0x6E
#define INVSENSOR_RA_MEM_R_W          0x6F
#define INVSENSOR_RA_DMP_CFG_1        0x70
#define INVSENSOR_RA_DMP_CFG_2        0x71
#define INVSENSOR_RA_FIFO_COUNTH      0x72
#define INVSENSOR_RA_FIFO_COUNTL      0x73
#define INVSENSOR_RA_FIFO_R_W         0x74
#define INVSENSOR_RA_WHO_AM_I         0x75
#define INVSENSOR_WHOAMI_20689        0x98

#define INVSENSOR_SMPLRT_DIV      0       // 8000Hz

#define BIT_GYRO                    3
#define BIT_ACC                     2
#define BIT_TEMP                    1

#define INVSENSOR_LPF_256HZ       0
#define INVSENSOR_LPF_188HZ       1
#define INVSENSOR_LPF_98HZ        2
#define INVSENSOR_LPF_42HZ        3
#define INVSENSOR_LPF_20HZ        4
#define INVSENSOR_LPF_10HZ        5
#define INVSENSOR_LPF_5HZ         6

#define INVSENSOR_A_2MG                ((float)0.00006103f)  //g/LSB
#define INVSENSOR_A_4MG                ((float)0.00012207f)  //g/LSB
#define INVSENSOR_A_8MG                ((float)0.00024414f)  //g/LSB

#define INVSENSOR_G_S250DPS            ((float)0.0076296f)  //dps/LSB
#define INVSENSOR_G_S500DPS            ((float)0.0152592f)  //dps/LSB
#define INVSENSOR_G_S1000DPS        	 ((float)0.0305185f)  //dps/LSB
#define INVSENSOR_G_S2000DPS           ((float)0.0610370f)  //dps/LSB

/**********************************************************************************************************
*函 数 名: spiWriteInvSensor
*功能说明: spi写
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
uint8_t spiWriteInvSensor( uint8_t reg, uint8_t data )
{
	HAL_GPIO_WritePin( GPIOC, MPU_CS_Pin, GPIO_PIN_RESET );
	HAL_SPI_Transmit( &hspi2, &reg, 1, 1500 );
	if ( HAL_SPI_Transmit( &hspi2, &data, 1, 1500 ) != HAL_OK )
	{
		return(0);
	}
	HAL_GPIO_WritePin( GPIOC, MPU_CS_Pin, GPIO_PIN_SET );
	return(1);
}

/**********************************************************************************************************
*函 数 名: spiReadInvSensor
*功能说明: spi读
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
uint8_t spiReadInvSensor( uint8_t reg, uint8_t *data, uint8_t Size )
{
	HAL_GPIO_WritePin( GPIOC, MPU_CS_Pin, GPIO_PIN_RESET );
	reg = reg | 0x80;
	HAL_SPI_Transmit( &hspi2, &reg, 1, 1500 );
	if ( HAL_SPI_Receive( &hspi2, data, Size, 1500 ) != HAL_OK )
	{
		return(0);
	}
	HAL_GPIO_WritePin( GPIOC, MPU_CS_Pin, GPIO_PIN_SET ); /* HAL_SPI_TransmitReceive(&hspi1,&reg,&reg_val,1,1500); */
	return(1);
}

/**********************************************************************************************************
*函 数 名: ICM20689_Detect
*功能说明: 检测ICM20689是否存在
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
bool invSensorDetect(void)
{
    uint8_t whoAmI;
		uint8_t i=50;
		spiWriteInvSensor( INVSENSOR_RA_PWR_MGMT_1, 0x80 );
		HAL_Delay( 50 );
		spiWriteInvSensor( INVSENSOR_RA_PWR_MGMT_1, 0x01 );
		HAL_Delay( 50 );
		spiReadInvSensor(INVSENSOR_RA_WHO_AM_I, &whoAmI, 1);
    while(whoAmI != INVSENSOR_WHOAMI_20689)
		{
			spiReadInvSensor(INVSENSOR_RA_WHO_AM_I, &whoAmI, 1);
			i--;
			if(i==0)
			{
				return false;
			}
		}
    return true;
}

/**********************************************************************************************************
*函 数 名: ICM20689_Init
*功能说明: ICM20689寄存器配置初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void invSensorInit(void)
{
			HAL_Delay( 100 );
			spiWriteInvSensor( INVSENSOR_RA_PWR_MGMT_1, 0x80 );
			HAL_Delay( 50 );
			spiWriteInvSensor( INVSENSOR_RA_PWR_MGMT_1, 0x01 );
			HAL_Delay( 50 );
			invSensorDetect();
			/*复位reg*/
			spiWriteInvSensor( INVSENSOR_RA_SIGNAL_PATH_RESET, 0x03 );
			HAL_Delay( 10 );
			/*复位reg*/
			spiWriteInvSensor( INVSENSOR_RA_USER_CTRL, 0x01 );
			HAL_Delay( 10 );
			spiWriteInvSensor( 0x70, 0x40 );//DMP
			HAL_Delay( 10 );
			spiWriteInvSensor( INVSENSOR_RA_PWR_MGMT_2, 0x00 );
			HAL_Delay( 10 );
			spiWriteInvSensor( INVSENSOR_RA_SMPLRT_DIV, 0 );
			HAL_Delay( 10 );
			spiWriteInvSensor( INVSENSOR_RA_CONFIG, INVSENSOR_LPF_20HZ );
			HAL_Delay( 10 );
			spiWriteInvSensor( INVSENSOR_RA_GYRO_CONFIG, (3 << 3) );  
			HAL_Delay( 10 );
			spiWriteInvSensor( INVSENSOR_RA_ACCEL_CONFIG, (2 << 3));  
			HAL_Delay( 10 );
			/*加速度计LPF 20HZ*/
			spiWriteInvSensor( 0X1D, 0x04 );
			HAL_Delay( 10 );
			/*关闭低功耗*/
			spiWriteInvSensor( 0X1E, 0x00 );
			HAL_Delay( 10 );
			/*关闭FIFO*/
			spiWriteInvSensor( 0X23, 0x00 );
			HAL_Delay( 10 );
}

/**********************************************************************************************************
*函 数 名: invSensorRead
*功能说明: ICM20689读取
*形    参: 读出数据指针
*返 回 值: 无
**********************************************************************************************************/
uint8_t invSensorBuffer[14];
void invSensorRead(void)
{
	uint8_t Status = 0;
	while ( !Status )
		Status = spiReadInvSensor( INVSENSOR_RA_ACCEL_XOUT_H, invSensorBuffer,14);
}

/**********************************************************************************************************
*函 数 名: ICM20689_ReadAcc
*功能说明: ICM20689读取加速度传感器，并转化为标准单位
*形    参: 读出数据指针
*返 回 值: 无
**********************************************************************************************************/
void invSensorReadAccel(Vector3ForFloat* acc)
{
    Vector3ForInt16 accRaw;
    accRaw.x = (invSensorBuffer[0] << 8) | invSensorBuffer[1];
    accRaw.y = (invSensorBuffer[2] << 8) | invSensorBuffer[3]; 
    accRaw.z = (invSensorBuffer[4] << 8) | invSensorBuffer[5]; 

    //统一传感器坐标系（并非定义安装方向）
    accRaw.x = accRaw.x;
    accRaw.y = accRaw.y;
    accRaw.z = accRaw.z;

    acc->x = (float)accRaw.y * INVSENSOR_A_8MG;
    acc->y = (float)accRaw.x * INVSENSOR_A_8MG;
    acc->z = (float)accRaw.z * INVSENSOR_A_8MG;
}

/**********************************************************************************************************
*函 数 名: ICM20689_ReadGyro
*功能说明: ICM20689读取陀螺仪传感器，并转化为标准单位
*形    参: 读出数据指针
*返 回 值: 无
**********************************************************************************************************/
void invSensorReadGyro(Vector3ForFloat* gyro)
{
    Vector3ForInt16 gyroData;
    gyroData.x = (invSensorBuffer[8] << 8) | invSensorBuffer[9];
    gyroData.y = (invSensorBuffer[10] << 8) | invSensorBuffer[11];
    gyroData.z = (invSensorBuffer[12] << 8) | invSensorBuffer[13];

    //统一传感器坐标系（并非定义安装方向）
    gyroData.x = gyroData.x;
    gyroData.y = gyroData.y;
    gyroData.z = -gyroData.z;

    gyro->x = gyroData.y * INVSENSOR_G_S2000DPS;
    gyro->y = gyroData.x * INVSENSOR_G_S2000DPS;
    gyro->z = gyroData.z * INVSENSOR_G_S2000DPS;
}

/**********************************************************************************************************
*函 数 名: ICM20689_ReadTemp
*功能说明: ICM20689读取温度传感器
*形    参: 读出数据指针
*返 回 值: 无
**********************************************************************************************************/
void invSensorReadTemperature(float* temp)
{
    static int16_t temperature_temp;
    temperature_temp = (invSensorBuffer[6] << 8) | invSensorBuffer[7];  
    *temp = 25 + (float)temperature_temp / 326.8f;
}


