#ifndef __PARAMETER_H__
#define __PARAMETER_H__

#include "board.h"

__packed struct Parameter_s
{
		uint16_t  FirstInit;
	  float GyroOffset[VECTOR_XYZ];
    float GyroScale[VECTOR_XYZ];
    float AccelOffset[VECTOR_XYZ];
    float AccelScale[VECTOR_XYZ];
    float MagOffset[VECTOR_XYZ];
    float MagScale[VECTOR_XYZ];
    float MagEarthMag;
    float ImuLevel[VECTOR_XYZ];
    float PidAngularVelocityLoop[VECTOR_EULER][VECTOR_PID];
		float PidAngleLoop[VECTOR_EULER][VECTOR_PID];
		float PidPositionVelocityLoop[VECTOR_XYZ][VECTOR_PID];
		float PidPositionLoop[VECTOR_XYZ][VECTOR_PID];
    float EscCalibrationFlag;
};

union Parameter
{
	/* 这里使用联合体，长度是4KByte，联合体内部是一个结构体，该结构体内是需要保存的参数 */
	struct Parameter_s	set;
	uint8_t			byte[4096];
};

extern union Parameter myParameters;

typedef struct
{
	uint8_t		SaveEnable;
	uint8_t		SaveTrig;
	uint16_t	TimeDelay;
}parameterStateStruct;

void ParamInit(void);
void ParamSaveToFlash(void);
void ParamUpdateData(uint8_t saveTrig);
void ParamGetData(uint16_t dataNum, void *data, uint8_t length);
const char* ParamGetString(uint8_t paramNum);

#endif

