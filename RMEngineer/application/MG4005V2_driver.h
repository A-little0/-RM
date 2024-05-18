#ifndef __MG4005_DRIVER_H
#define __MG4005_DRIVER_H

#include "main.h"
#include "can.h"

#define DEVICE_STD_ID			(0x140)//
#define DEVICE_STD_BOARDCAST_ID	(0x280)//
typedef struct {
	uint8_t anglePid_kp;
	uint8_t anglePid_ki;
	uint8_t speedPid_kp;
	uint8_t speedPid_ki;
	uint8_t iqPid_kp;
	uint8_t iqPid_ki;
}MG4005V2_PIDHandleTypedef;


typedef struct {
	//int16_t powerControl;//输出功率
	int16_t  iq;//转矩电流值
	int16_t  speed;//速度
	uint32_t Accel;//加速度
	uint16_t encoder;//编码器位置
	uint16_t encoderRaw;//编码器起始位置
	uint16_t encoderOffset;//编码器零偏
	
	int64_t  motorAngle;//电机多圈角度
	uint32_t circleAngle;//电机单圈角度
	
	int8_t 	 temperature;//电机温度
	uint16_t voltage;//电机电压
	uint8_t  errorState;//错误状态字节
	
	int16_t iA;//A相电流数据
	int16_t iB;//B相电流数据
	int16_t iC;//C相电流数据
	
	
}MG4005V2_DataHandleTypedef;


void ReadPIDArgument(uint8_t motorId);
void WritePIDArgumentToRAM(uint8_t motorId,MG4005V2_PIDHandleTypedef* mg_pid);
void WritePIDArgumentToROM(uint8_t motorId,MG4005V2_PIDHandleTypedef* mg_pid);
void ReadAccleration(uint8_t motorId);
void WriteAcclerationToRAM(uint8_t motorId,MG4005V2_DataHandleTypedef* mgmotorX);
void ReadEncoder(uint8_t motorId);
void WriteTargetEncoderToROM(uint8_t motorId,MG4005V2_DataHandleTypedef* mgmotorX);
void WriteNowEncoderToROM(uint8_t motorId);
void ReadMCircle(uint8_t motorId);
void ReadSCircle(uint8_t motorId);
void ClearMotorAngle(uint8_t motorId);
//
void ClearErrorMessage(uint8_t motorId);
void ReadMotorState2(uint8_t motorId);
void ReadMotorState3(uint8_t motorId);
void TurnOffMotor(uint8_t motorId);
void MotorStop(uint8_t motorId);
void MotorRun(uint8_t motorId);
void TorqueOpenControlLoop(uint8_t motorId,int16_t powerControl);
void TorqueCloseControlLoop(uint8_t motorId,int16_t iqControl);
void SpeedCloseControlLoop(uint8_t motorId,int32_t speedControl);
void AngleCloseControlLoop1(uint8_t motorId,int32_t angleControl);
void AngleCloseControlLoop2(uint8_t motorId,int32_t angleControl,uint16_t maxSpeed);
void AngleCloseControlLoop3(uint8_t motorId,uint16_t angleControl,uint8_t spinDirection);
void AngleCloseControlLoop4(uint8_t motorId,uint16_t angleControl,uint8_t spinDirection,uint16_t maxSpeed);
void AngleCloseControlLoop5(uint8_t motorId,int32_t angleControl);
void AngleCloseControlLoop6(uint8_t motorId,int32_t angleControl,uint16_t maxSpeed);

#endif
