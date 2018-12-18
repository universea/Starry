/* ---------------------------------------------------------------------
 * 
 * Spl06库函数:
 * 气压计操作函数.
 *
 *   修订操作      版本号       日期         修订人
 * ------------    ------    ----------    ------------
 *     创建        1.0       2018.01.19      Universea
 *
 * ---------------------------------------------------------------------*/
#include "spi.h"
#include "gpio.h"
#include "math.h"
#include "Spl06.h"

#define CONTINUOUS_PRESSURE     1
#define CONTINUOUS_TEMPERATURE  2
#define CONTINUOUS_P_AND_T      3
#define PRESSURE_SENSOR     0
#define TEMPERATURE_SENSOR  1
#define uint32 unsigned int

struct spl0601_calib_param_t {	
    int16_t c0;
    int16_t c1;
    int32_t c00;
    int32_t c10;
    int16_t c01;
    int16_t c11;
    int16_t c20;
    int16_t c21;
    int16_t c30;       
};

struct spl0601_t 
{	
    struct spl0601_calib_param_t calib_param;/**<calibration data*/	
    uint8_t 	chip_id; /**<chip id*/	
    int32_t 	i32rawPressure;
    int32_t 	i32rawTemperature;
    int32_t 	i32kP;    
    int32_t 	i32kT;
};

typedef struct {
		float BaroOffset;
		float Altitude_3;
		float Height;
		float BaroPressure;
		float BaroTemperature;
		float AltitudeHigh;
		unsigned char BaroStart;

} spl0601DataStruct;

static struct spl0601_t spl0601;
static struct spl0601_t *p_spl0601;
static spl0601DataStruct spl0601Data;

void spl0601GetCalibrationParamters ( void );

/**********************************************************************************************************
*函 数 名: spl06Write
*功能说明: spi写
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
uint8_t spl06Write( uint8_t reg, uint8_t data )
{
	HAL_GPIO_WritePin( GPIOC, BARO_CS_Pin, GPIO_PIN_RESET );
	HAL_SPI_Transmit( &hspi2, &reg, 1, 1500 );
	if ( HAL_SPI_Transmit( &hspi2, &data, 1, 1500 ) != HAL_OK )
	{
		return(0);
	}
	HAL_GPIO_WritePin( GPIOC, BARO_CS_Pin, GPIO_PIN_SET );
	return(1);
}

/**********************************************************************************************************
*函 数 名: spl06Read
*功能说明: spi读
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
uint8_t spl06Read( uint8_t reg )
{
  uint8_t reg_data;
	HAL_GPIO_WritePin( GPIOC, BARO_CS_Pin, GPIO_PIN_RESET );
	reg = reg | 0x80;
	HAL_SPI_Transmit( &hspi2, &reg, 1, 1500 );
	if ( HAL_SPI_Receive( &hspi2, &reg_data, 1, 1500 ) != HAL_OK )
	{
		
	}
	HAL_GPIO_WritePin( GPIOC, BARO_CS_Pin, GPIO_PIN_SET ); /* HAL_SPI_TransmitReceive(&hspi1,&reg,&reg_val,1,1500); */
        return(reg_data);
}
/**********************************************************************************************************
*函 数 名: spl0601RateSet
*功能说明: 设置温度传感器的每秒采样次数以及过采样率
*形    参: uint8 uint8_tOverSmpl  过采样率         Maximal = 128
*          uint8 uint8_tSmplRate  每秒采样次数(Hz) Maximal = 128
*          uint8 iSensor     0: Pressure; 1: Temperature
*返 回 值: 无
**********************************************************************************************************/
void spl0601RateSet ( uint8_t iSensor, uint8_t uint8_tSmplRate, uint8_t uint8_tOverSmpl )
{
    uint8_t reg = 0;
    int32_t i32kPkT = 0;
    switch ( uint8_tSmplRate )
    {
    case 2:
        reg |= ( 1 << 4 );
        break;
    case 4:
        reg |= ( 2 << 4 );
        break;
    case 8:
        reg |= ( 3 << 4 );
        break;
    case 16:
        reg |= ( 4 << 4 );
        break;
    case 32:
        reg |= ( 5 << 4 );
        break;
    case 64:
        reg |= ( 6 << 4 );
        break;
    case 128:
        reg |= ( 7 << 4 );
        break;
    case 1:
    default:
        break;
    }
    switch ( uint8_tOverSmpl )
    {
    case 2:
        reg |= 1;
        i32kPkT = 1572864;
        break;
    case 4:
        reg |= 2;
        i32kPkT = 3670016;
        break;
    case 8:
        reg |= 3;
        i32kPkT = 7864320;
        break;
    case 16:
        i32kPkT = 253952;
        reg |= 4;
        break;
    case 32:
        i32kPkT = 516096;
        reg |= 5;
        break;
    case 64:
        i32kPkT = 1040384;
        reg |= 6;
        break;
    case 128:
        i32kPkT = 2088960;
        reg |= 7;
        break;
    case 1:
    default:
        i32kPkT = 524288;
        break;
    }

    if ( iSensor == 0 )
    {
        p_spl0601->i32kP = i32kPkT;
        spl06Write ( 0x06, reg );
        if ( uint8_tOverSmpl > 8 )
        {
            reg = spl06Read ( 0x09 );
            spl06Write ( 0x09, reg | 0x04 );
        }
    }
    if ( iSensor == 1 )
    {
        p_spl0601->i32kT = i32kPkT;
        spl06Write ( 0x07, reg | 0x80 ); //Using mems temperature
        if ( uint8_tOverSmpl > 8 )
        {
            reg = spl06Read ( 0x09 );
            spl06Write ( 0x09, reg | 0x08 );
        }
    }

}

/**********************************************************************************************************
*函 数 名: spl0601GetCalibrationParamters
*功能说明: 获取校准参数
*形    参: 
*返 回 值: 无
**********************************************************************************************************/
void spl0601GetCalibrationParamters ( void )
{
    uint32 h;
    uint32 m;
    uint32 l;
    h =  spl06Read ( 0x10 );
    l  =  spl06Read ( 0x11 );
    p_spl0601->calib_param.c0 = ( int16_t ) h << 4 | l >> 4;
    p_spl0601->calib_param.c0 = ( p_spl0601->calib_param.c0 & 0x0800 ) ? ( 0xF000 | p_spl0601->calib_param.c0 ) : p_spl0601->calib_param.c0;
    h =  spl06Read ( 0x11 );
    l  =  spl06Read ( 0x12 );
    p_spl0601->calib_param.c1 = ( int16_t ) ( h & 0x0F ) << 8 | l;
    p_spl0601->calib_param.c1 = ( p_spl0601->calib_param.c1 & 0x0800 ) ? ( 0xF000 | p_spl0601->calib_param.c1 ) : p_spl0601->calib_param.c1;
    h =  spl06Read ( 0x13 );
    m =  spl06Read ( 0x14 );
    l =  spl06Read ( 0x15 );
    p_spl0601->calib_param.c00 = ( int32_t ) h << 12 | ( int32_t ) m << 4 | ( int32_t ) l >> 4;
    p_spl0601->calib_param.c00 = ( p_spl0601->calib_param.c00 & 0x080000 ) ? ( 0xFFF00000 | p_spl0601->calib_param.c00 ) : p_spl0601->calib_param.c00;
    h =  spl06Read ( 0x15 );
    m =  spl06Read ( 0x16 );
    l =  spl06Read ( 0x17 );
    p_spl0601->calib_param.c10 = ( int32_t ) h << 16 | ( int32_t ) m << 8 | l;
    p_spl0601->calib_param.c10 = ( p_spl0601->calib_param.c10 & 0x080000 ) ? ( 0xFFF00000 | p_spl0601->calib_param.c10 ) : p_spl0601->calib_param.c10;
    h =  spl06Read ( 0x18 );
    l  =  spl06Read ( 0x19 );
    p_spl0601->calib_param.c01 = ( int16_t ) h << 8 | l;
    h =  spl06Read ( 0x1A );
    l  =  spl06Read ( 0x1B );
    p_spl0601->calib_param.c11 = ( int16_t ) h << 8 | l;
    h =  spl06Read ( 0x1C );
    l  =  spl06Read ( 0x1D );
    p_spl0601->calib_param.c20 = ( int16_t ) h << 8 | l;
    h =  spl06Read ( 0x1E );
    l  =  spl06Read ( 0x1F );
    p_spl0601->calib_param.c21 = ( int16_t ) h << 8 | l;
    h =  spl06Read ( 0x20 );
    l  =  spl06Read ( 0x21 );
    p_spl0601->calib_param.c30 = ( int16_t ) h << 8 | l;
}

/**********************************************************************************************************
*函 数 名: spl0601StartTemperature
*功能说明: 发起一次温度测量
*形    参: 
*返 回 值: 无
**********************************************************************************************************/
void spl0601StartTemperature ( void )
{
    spl06Write ( 0x08, 0x02 );
}

/**********************************************************************************************************
*函 数 名: spl0601StartPressure
*功能说明: 发起一次压力值测量
*形    参: 
*返 回 值: 无
**********************************************************************************************************/
void spl0601StartPressure ( void )
{
    spl06Write ( 0x08, 0x01 );
}

/**********************************************************************************************************
*函 数 名: spl0601StartContinuous
*功能说明: Select node for the continuously measurement
*形    参: uint8 mode  1: pressure; 2: temperature; 3: pressure and temperature
*返 回 值: 无
**********************************************************************************************************/
void spl0601StartContinuous ( uint8_t mode )
{
    spl06Write ( 0x08, mode + 4 );
}

/**********************************************************************************************************
*函 数 名: spl0601GetRawTemperature
*功能说明: 获取温度的原始值，并转换成32Bits整数
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void spl0601GetRawTemperature ( void )
{
    uint8_t h[3] = {0};

    h[0] = spl06Read ( 0x03 );
    h[1] = spl06Read ( 0x04 );
    h[2] = spl06Read ( 0x05 );

    p_spl0601->i32rawTemperature = ( int32_t ) h[0] << 16 | ( int32_t ) h[1] << 8 | ( int32_t ) h[2];
    p_spl0601->i32rawTemperature = ( p_spl0601->i32rawTemperature & 0x800000 ) ? ( 0xFF000000 | p_spl0601->i32rawTemperature ) : p_spl0601->i32rawTemperature;
}

/**********************************************************************************************************
*函 数 名: spl0601GetRawPressure
*功能说明: 获取压力原始值，并转换成32bits整数
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void spl0601GetRawPressure ( void )
{
    uint8_t h[3];

    h[0] = spl06Read ( 0x00 );
    h[1] = spl06Read ( 0x01 );
    h[2] = spl06Read ( 0x02 );

    p_spl0601->i32rawPressure = ( int32_t ) h[0] << 16 | ( int32_t ) h[1] << 8 | ( int32_t ) h[2];
    p_spl0601->i32rawPressure = ( p_spl0601->i32rawPressure & 0x800000 ) ? ( 0xFF000000 | p_spl0601->i32rawPressure ) : p_spl0601->i32rawPressure;
}

/**********************************************************************************************************
*函 数 名: spl0601Init
*功能说明: SPL06-01 初始化函数
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void spl0601Init (void)
{
    p_spl0601 = &spl0601; /* read Chip Id */
    p_spl0601->i32rawPressure = 0;
    p_spl0601->i32rawTemperature = 0;
    p_spl0601->chip_id = spl06Read ( 0x0D );// 0x34  0x10
	
    spl0601GetCalibrationParamters();

    spl0601RateSet ( PRESSURE_SENSOR, 128, 16 );

    spl0601RateSet ( TEMPERATURE_SENSOR, 8, 8 );

    spl0601StartContinuous ( CONTINUOUS_P_AND_T );
}

/**********************************************************************************************************
*函 数 名: spl0601GetTemperature
*功能说明: 在获取原始值的基础上，返回浮点校准后的温度值
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
float spl0601GetTemperature ( void )
{
    float fTCompensate;
    float fTsc;

    fTsc = p_spl0601->i32rawTemperature / ( float ) p_spl0601->i32kT;
    fTCompensate =  p_spl0601->calib_param.c0 * 0.5 + p_spl0601->calib_param.c1 * fTsc;
    return fTCompensate;
}

/**********************************************************************************************************
*函 数 名: spl0601GetPressure
*功能说明: 在获取原始值的基础上，返回浮点校准后的压力值
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
float spl0601GetPressure ( void )
{
    float fTsc, fPsc;
    float qua2, qua3;
    float fPCompensate;

    fTsc = p_spl0601->i32rawTemperature / ( float ) p_spl0601->i32kT;
    fPsc = p_spl0601->i32rawPressure / ( float ) p_spl0601->i32kP;
    qua2 = p_spl0601->calib_param.c10 + fPsc * ( p_spl0601->calib_param.c20 + fPsc * p_spl0601->calib_param.c30 );
    qua3 = fTsc * fPsc * ( p_spl0601->calib_param.c11 + fPsc * p_spl0601->calib_param.c21 );
    //qua3 = 0.9f *fTsc * fPsc * (p_spl0601->calib_param.c11 + fPsc * p_spl0601->calib_param.c21);

    fPCompensate = p_spl0601->calib_param.c00 + fPsc * qua2 + fTsc * p_spl0601->calib_param.c01 + qua3;
    //fPCompensate = p_spl0601->calib_param.c00 + fPsc * qua2 + 0.9f *fTsc  * p_spl0601->calib_param.c01 + qua3;
    return fPCompensate;
}

/**********************************************************************************************************
*函 数 名: spl0601Detect
*功能说明: 检测spl0601是否存在
*形    参: 无
*返 回 值: 存在状态
**********************************************************************************************************/
bool spl0601Detect(void)
{
		int i=10;
		while(i--)
		{
				p_spl0601->chip_id = spl06Read ( 0x0D );// 0x34  0x10
				if(p_spl0601->chip_id == 0x10)
					return true;
		}
    return false;
}



/**********************************************************************************************************
*函 数 名: spl0601Update
*功能说明: spl0601数据更新
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void spl0601Update(void)
{
		spl0601GetRawTemperature();
    spl0601Data.BaroTemperature = spl0601GetTemperature();
    spl0601GetRawPressure();
    spl0601Data.BaroPressure = spl0601GetPressure();
	  spl0601Data.Altitude_3 = ( 101000 - spl0601Data.BaroPressure ) / 1000.0f;
    spl0601Data.Height = 0.82f * spl0601Data.Altitude_3 * spl0601Data.Altitude_3 * spl0601Data.Altitude_3 + 0.09f * ( 101000 - spl0601Data.BaroPressure ) * 100.0f ;
    //spl0601Data.AltitudeHigh = ( spl0601Data.Height - spl0601Data.BaroOffset ) ;
	  spl0601Data.AltitudeHigh = ((1.0f - pow(spl0601Data.BaroPressure / 101325.0f, 0.190295f)) * 4433000.0f);
}

/**********************************************************************************************************
*函 数 名: spl0601Read
*功能说明: 读取气压数据
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void spl0601Read (int32_t* baroAlt)
{
		*baroAlt = spl0601Data.Height;
}
/**********************************************************************************************************
*函 数 名: MS5611_ReadTemp
*功能说明: 读取MS5611的温度值
*形    参: 读出数据指针
*返 回 值: 无
**********************************************************************************************************/
void spl0601ReadTemperature(float* temp)
{
    *temp = (float)spl0601Data.BaroTemperature * 0.01f;
}
