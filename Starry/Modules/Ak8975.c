/* ---------------------------------------------------------------------
 * 
 * Ak8975库函数:
 * 磁力计操作函数.
 *
 *   修订操作      版本号       日期         修订人
 * ------------    ------    ----------    ------------
 *     创建        1.0       2018.01.19      Universea
 *
 * ---------------------------------------------------------------------*/
#include "Ak8975.h"
#include "spi.h"
#include "gpio.h"
#include "math.h"

#define AK8975_WIA_REG          0X00 
#define AK8975_INFO_REG         0X01 
#define AK8975_ST1_REG          0X02 
#define AK8975_HXL_REG          0X03 
#define AK8975_HXH_REG          0X04
#define AK8975_HYL_REG          0X05
#define AK8975_HYH_REG          0X06
#define AK8975_HZL_REG          0X07
#define AK8975_HZH_REG          0X08
#define AK8975_ST2_REG          0X09 
#define AK8975_CNTL_REG         0X0A 
#define AK8975_RSV_REG          0X0B
#define AK8975_ASTC_REG         0X0C 
#define AK8975_TS1_REG          0X0D
#define AK8975_TS2_REG          0X0E
#define AK8975_I2CDIS_REG       0X0F 
#define AK8975_ASAX_REG         0X10 
#define AK8975_ASAY_REG         0X11
#define AK8975_ASAZ_REG         0X12  

#define AK8975_MAG_TO_GAUSS 0.3*0.001 //uT/LSB

/**********************************************************************************************************
*函 数 名: spiWriteMag
*功能说明: spi写
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
uint8_t spiWriteMag( uint8_t reg, uint8_t data )
{
	HAL_GPIO_WritePin( GPIOC, MAG_CS_Pin, GPIO_PIN_RESET );
	HAL_SPI_Transmit( &hspi2, &reg, 1, 1500 );
	if ( HAL_SPI_Transmit( &hspi2, &data, 1, 1500 ) != HAL_OK )
	{
		return(0);
	}
	HAL_GPIO_WritePin( GPIOC, MAG_CS_Pin, GPIO_PIN_SET );
	return(1);
}

/**********************************************************************************************************
*函 数 名: spiReadMag
*功能说明: spi读
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
uint8_t spiReadMag( uint8_t reg, uint8_t *data, uint8_t Size )
{
	HAL_GPIO_WritePin( GPIOC, MAG_CS_Pin, GPIO_PIN_RESET );
	reg = reg | 0x80;
	HAL_SPI_Transmit( &hspi2, &reg, 1, 1500 );
	if ( HAL_SPI_Receive( &hspi2, data, Size, 1500 ) != HAL_OK )
	{
		return(0);
	}
	HAL_GPIO_WritePin( GPIOC, MAG_CS_Pin, GPIO_PIN_SET ); /* HAL_SPI_TransmitReceive(&hspi1,&reg,&reg_val,1,1500); */
	return(1);
}

/**********************************************************************************************************
*函 数 名: ak8975Detect
*功能说明: 检测AK8975是否存在
*形    参: 无
*返 回 值: 存在状态
**********************************************************************************************************/
bool ak8975Detect(void)
{
		uint8_t whoAmI;
		spiWriteMag(AK8975_CNTL_REG, 0x01);
		spiReadMag(AK8975_WIA_REG,&whoAmI,1);
    if(whoAmI == 0x48)
        return true;
    else
        return false;
}

/**********************************************************************************************************
*函 数 名: ak8975Init
*功能说明: ak8975寄存器配置初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void ak8975Init(void)
{
		spiWriteMag( AK8975_CNTL_REG, 0x01 );
}
/**********************************************************************************************************
*函 数 名: trigAk8975
*功能说明: AK8975单次触发
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
static void trigAk8975( void )
{
	spiWriteMag( AK8975_CNTL_REG, 0x01 );
}

Vector3ForInt16 magData;
/**********************************************************************************************************
*函 数 名: ak8975Update
*功能说明: AK8975数据更新
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void ak8975Update(void)
{
    uint8_t buffer[6];
		spiReadMag(AK8975_HXL_REG,buffer,6);
		trigAk8975();
		magData.x	= ( ( ( (int16_t) buffer[1]) << 8) | buffer[0]);
		magData.y	= ( ( ( (int16_t) buffer[3]) << 8) | buffer[2]);
		magData.z	= ( ( ( (int16_t) buffer[5]) << 8) | buffer[4]);
	
    //统一传感器坐标系（并非定义安装方向）
    magData.x = magData.x;
    magData.y = magData.y;
    magData.z = -magData.z;
}

/**********************************************************************************************************
*函 数 名: ak8975Read
*功能说明: 读取地磁传感器数据,并转换为标准单位
*形    参: 读出数据指针
*返 回 值: 无
**********************************************************************************************************/
void ak8975Read(Vector3ForFloat* mag)
{
    mag->x = magData.x * AK8975_MAG_TO_GAUSS;
    mag->y = magData.y * AK8975_MAG_TO_GAUSS;
    mag->z = magData.z * AK8975_MAG_TO_GAUSS;
}
