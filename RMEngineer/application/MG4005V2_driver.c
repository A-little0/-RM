/**
  ****************************(C) COPYRIGHT 2024 征途****************************
  * @file       MG4005V2_driver.c/h
  * @brief      
  *            
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Fbe-26-2024    chenjiangnan    
  *
  @verbatim
  ==============================================================================
--------------------------MG4005V2 COONTROL COMMAND------------------------------
	0.读取pid参数命令	 													0x30
	1.写pid参数到RAM命令 													0x31
	2.写pid参数到ROM命令 													0x32
	3.读取加速度命令 														0x33
	4.写入加速度到RAM命令 													0x34
	5.读取加速度命令 														0x90
	6.读取编码器的命令 														0x91
	7.写入编码器的值到ROM作为电机零点命令  									0x91
	8.写入当前位置到ROM作为电机零点命令 										0x19
	9.读取多圈角度命令 														0x92
	10.读取单圈角度命令 														0x94
	11.清除电机角度命令（设置电机初始位置）									0x95
	12.清除电机状态1和和错误标志命令											0x9A
	12.读取电机错误标志命令 													0x9B
	13.读取电机状态2命令														0x9C
	14.读取电机状态3命令														0x9D
	15.电机关闭命令															0x80
	16.电机停止命令															0x81
	17.电机运行命令															0x88
	18.转矩开环控制命令														0xA0
	19.转矩闭环控制命令														0xA1
	20.速度闭环控制命令														0xA2
	21.位置闭环控制命令1														0xA3
	22.位置闭环控制命令2														0xA4
	23.位置闭环控制命令3														0xA5
	21.位置闭环控制命令4														0xA6
	22.位置闭环控制命令5														0xA7
	23.位置闭环控制命令6														0xA8
  ==============================================================================
  @endverbatim
 *****************************(C) COPYRIGHT 2024 征途****************************/
#include "MG4005V2_driver.h"


extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

CAN_TxHeaderTypeDef canTxHeader;
CAN_RxHeaderTypeDef canRxHeader;

uint8_t canTxData[8];
uint8_t canRxData[8];


void ReadPIDArgument(uint8_t motorId)
{
		uint32_t txMailBox;
		canTxHeader.StdId = DEVICE_STD_ID + motorId;
		canTxHeader.ExtId = 0x00;
		canTxHeader.RTR = CAN_RTR_DATA;
		canTxHeader.IDE = CAN_ID_STD;
		canTxHeader.DLC = 8;
		
		canTxData[0] = 0x30;
		canTxData[1] = 0x00;
		canTxData[2] = 0x00;
		canTxData[3] = 0x00;
		canTxData[4] = 0x00;
		canTxData[5] = 0x00;
		canTxData[6] = 0x00;
		canTxData[7] = 0x00;
		
		/* send can command */
		HAL_CAN_AddTxMessage(&hcan2, &canTxHeader, canTxData, &txMailBox);
		
}
void WritePIDArgumentToRAM(uint8_t motorId,MG4005V2_PIDHandleTypedef* mg_pid)
{
		uint32_t txMailBox;
		canTxHeader.StdId = DEVICE_STD_ID + motorId;
		canTxHeader.ExtId = 0x00;
		canTxHeader.RTR = CAN_RTR_DATA;
		canTxHeader.IDE = CAN_ID_STD;
		canTxHeader.DLC = 8;
		
		canTxData[0] = 0x31;
		canTxData[1] = 0x00;
		canTxData[2] = mg_pid->anglePid_kp;
		canTxData[3] = mg_pid->anglePid_ki;
		canTxData[4] = mg_pid->speedPid_kp;
		canTxData[5] = mg_pid->speedPid_ki;
		canTxData[6] = mg_pid->iqPid_kp;
		canTxData[7] = mg_pid->iqPid_ki;
		
		/* send can command */
		HAL_CAN_AddTxMessage(&hcan2, &canTxHeader, canTxData, &txMailBox);
		
}
void WritePIDArgumentToROM(uint8_t motorId,MG4005V2_PIDHandleTypedef* mg_pid)
{
		uint32_t txMailBox;
		canTxHeader.StdId = DEVICE_STD_ID + motorId;
		canTxHeader.ExtId = 0x00;
		canTxHeader.RTR = CAN_RTR_DATA;
		canTxHeader.IDE = CAN_ID_STD;
		canTxHeader.DLC = 8;
		
		canTxData[0] = 0x32;
		canTxData[1] = 0x00;
		canTxData[2] = mg_pid->anglePid_kp;
		canTxData[3] = mg_pid->anglePid_ki;
		canTxData[4] = mg_pid->speedPid_kp;
		canTxData[5] = mg_pid->speedPid_ki;
		canTxData[6] = mg_pid->iqPid_kp;
		canTxData[7] = mg_pid->iqPid_ki;
		
		
		/* send can command */
		HAL_CAN_AddTxMessage(&hcan2, &canTxHeader, canTxData, &txMailBox);
	
}	
void ReadAccleration(uint8_t motorId)
{
		uint32_t txMailBox;
		canTxHeader.StdId = DEVICE_STD_ID + motorId;
		canTxHeader.ExtId = 0x00;
		canTxHeader.RTR = CAN_RTR_DATA;
		canTxHeader.IDE = CAN_ID_STD;
		canTxHeader.DLC = 8;
		
		canTxData[0] = 0x33;
		canTxData[1] = 0x00;
		canTxData[2] = 0x00;
		canTxData[3] = 0x00;
		canTxData[4] = 0x00;
		canTxData[5] = 0x00;
		canTxData[6] = 0x00;
		canTxData[7] = 0x00;
		
		/* send can command */
		HAL_CAN_AddTxMessage(&hcan2, &canTxHeader, canTxData, &txMailBox);
	
}
void WriteAcclerationToRAM(uint8_t motorId,MG4005V2_DataHandleTypedef* mgmotorX)
{
		uint32_t txMailBox;
		canTxHeader.StdId = DEVICE_STD_ID + motorId;
		canTxHeader.ExtId = 0x00;
		canTxHeader.RTR = CAN_RTR_DATA;
		canTxHeader.IDE = CAN_ID_STD;
		canTxHeader.DLC = 8;
		
		canTxData[0] = 0x34;
		canTxData[1] = 0x00;
		canTxData[2] = 0x00;
		canTxData[3] = 0x00;
		canTxData[4] = *((uint8_t *)&mgmotorX->Accel + 0);
		canTxData[5] = *((uint8_t *)&mgmotorX->Accel + 1);
		canTxData[6] = *((uint8_t *)&mgmotorX->Accel + 2);
		canTxData[7] = *((uint8_t *)&mgmotorX->Accel + 3);
		
		/* send can command */
		HAL_CAN_AddTxMessage(&hcan2, &canTxHeader, canTxData, &txMailBox);
	
}
void ReadEncoder(uint8_t motorId)
{
		uint32_t txMailBox;
		canTxHeader.StdId = DEVICE_STD_ID + motorId;
		canTxHeader.ExtId = 0x00;
		canTxHeader.RTR = CAN_RTR_DATA;
		canTxHeader.IDE = CAN_ID_STD;
		canTxHeader.DLC = 8;
		
		canTxData[0] = 0x90;
		canTxData[1] = 0x00;
		canTxData[2] = 0x00;
		canTxData[3] = 0x00;
		canTxData[4] = 0x00;
		canTxData[5] = 0x00;
		canTxData[6] = 0x00;
		canTxData[7] = 0x00;
		
		/* send can command */
		HAL_CAN_AddTxMessage(&hcan2, &canTxHeader, canTxData, &txMailBox);
	
}
void WriteTargetEncoderToROM(uint8_t motorId,MG4005V2_DataHandleTypedef* mgmotorX)
{
		uint32_t txMailBox;
		canTxHeader.StdId = DEVICE_STD_ID + motorId;
		canTxHeader.ExtId = 0x00;
		canTxHeader.RTR = CAN_RTR_DATA;
		canTxHeader.IDE = CAN_ID_STD;
		canTxHeader.DLC = 8;
		
		canTxData[0] = 0x91;
		canTxData[1] = 0x00;
		canTxData[2] = 0x00;
		canTxData[3] = 0x00;
		canTxData[4] = 0x00;
		canTxData[5] = 0x00;
		canTxData[6] = *((uint8_t *)&mgmotorX->encoderOffset + 0);
		canTxData[7] = *((uint8_t *)&mgmotorX->encoderOffset + 1);
		
		/* send can command */
		HAL_CAN_AddTxMessage(&hcan2, &canTxHeader, canTxData, &txMailBox);
	
}
void WriteNowEncoderToROM(uint8_t motorId)
{
		uint32_t txMailBox;
		canTxHeader.StdId = DEVICE_STD_ID + motorId;
		canTxHeader.ExtId = 0x00;
		canTxHeader.RTR = CAN_RTR_DATA;
		canTxHeader.IDE = CAN_ID_STD;
		canTxHeader.DLC = 8;
		
		canTxData[0] = 0x19;
		canTxData[1] = 0x00;
		canTxData[2] = 0x00;
		canTxData[3] = 0x00;
		canTxData[4] = 0x00;
		canTxData[5] = 0x00;
		canTxData[6] = 0x00;
		canTxData[7] = 0x00;
		
		/* send can command */
		HAL_CAN_AddTxMessage(&hcan2, &canTxHeader, canTxData, &txMailBox);
	
}
void ReadMCircle(uint8_t motorId)
{
		uint32_t txMailBox;
		canTxHeader.StdId = DEVICE_STD_ID + motorId;
		canTxHeader.ExtId = 0x00;
		canTxHeader.RTR = CAN_RTR_DATA;
		canTxHeader.IDE = CAN_ID_STD;
		canTxHeader.DLC = 8;
		
		canTxData[0] = 0x92;
		canTxData[1] = 0x00;
		canTxData[2] = 0x00;
		canTxData[3] = 0x00;
		canTxData[4] = 0x00;
		canTxData[5] = 0x00;
		canTxData[6] = 0x00;
		canTxData[7] = 0x00;
		
		/* send can command */
		HAL_CAN_AddTxMessage(&hcan2, &canTxHeader, canTxData, &txMailBox);
	
}
void ReadSCircle(uint8_t motorId)
{
		uint32_t txMailBox;
		canTxHeader.StdId = DEVICE_STD_ID + motorId;
		canTxHeader.ExtId = 0x00;
		canTxHeader.RTR = CAN_RTR_DATA;
		canTxHeader.IDE = CAN_ID_STD;
		canTxHeader.DLC = 8;
		
		canTxData[0] = 0x94;
		canTxData[1] = 0x00;
		canTxData[2] = 0x00;
		canTxData[3] = 0x00;
		canTxData[4] = 0x00;
		canTxData[5] = 0x00;
		canTxData[6] = 0x00;
		canTxData[7] = 0x00;
		
		/* send can command */
		HAL_CAN_AddTxMessage(&hcan2, &canTxHeader, canTxData, &txMailBox);
	

}
//void ClearMotorAngle(uint8_t motorId)//暂未实现
//{
//		uint32_t txMailBox;
//		canTxHeader.StdId = DEVICE_STD_ID + motorId;
//		canTxHeader.ExtId = 0x00;
//		canTxHeader.RTR = CAN_RTR_DATA;
//		canTxHeader.IDE = CAN_ID_STD;
//		canTxHeader.DLC = 8;
//		
//		canTxData[0] = 0x95;
//		canTxData[1] = 0x00;
//		canTxData[2] = 0x00;
//		canTxData[3] = 0x00;
//		canTxData[4] = 0x00;
//		canTxData[5] = 0x00;
//		canTxData[6] = 0x00;
//		canTxData[7] = 0x00;
//		
//		/* send can command */
//		HAL_CAN_AddTxMessage(&hcan, &canTxHeader, canTxData, &txMailBox);
//	

//}
//12.清除电机状态1和和错误标志命令		
void ClearErrorMessage(uint8_t motorId)
{
		uint32_t txMailBox;
		canTxHeader.StdId = DEVICE_STD_ID + motorId;
		canTxHeader.ExtId = 0x00;
		canTxHeader.RTR = CAN_RTR_DATA;
		canTxHeader.IDE = CAN_ID_STD;
		canTxHeader.DLC = 8;
		
		canTxData[0] = 0x9B;
		canTxData[1] = 0x00;
		canTxData[2] = 0x00;
		canTxData[3] = 0x00;
		canTxData[4] = 0x00;
		canTxData[5] = 0x00;
		canTxData[6] = 0x00;
		canTxData[7] = 0x00;
		
		/* send can command */
		HAL_CAN_AddTxMessage(&hcan2, &canTxHeader, canTxData, &txMailBox);
	

}
void ReadMotorState2(uint8_t motorId)
{
		uint32_t txMailBox;
		canTxHeader.StdId = DEVICE_STD_ID + motorId;
		canTxHeader.ExtId = 0x00;
		canTxHeader.RTR = CAN_RTR_DATA;
		canTxHeader.IDE = CAN_ID_STD;
		canTxHeader.DLC = 8;
		
		canTxData[0] = 0x9C;
		canTxData[1] = 0x00;
		canTxData[2] = 0x00;
		canTxData[3] = 0x00;
		canTxData[4] = 0x00;
		canTxData[5] = 0x00;
		canTxData[6] = 0x00;
		canTxData[7] = 0x00;
		
		/* send can command */
		HAL_CAN_AddTxMessage(&hcan2, &canTxHeader, canTxData, &txMailBox);
	
}
void ReadMotorState3(uint8_t motorId)
{
		uint32_t txMailBox;
		canTxHeader.StdId = DEVICE_STD_ID + motorId;
		canTxHeader.ExtId = 0x00;
		canTxHeader.RTR = CAN_RTR_DATA;
		canTxHeader.IDE = CAN_ID_STD;
		canTxHeader.DLC = 8;
		
		canTxData[0] = 0x9D;
		canTxData[1] = 0x00;
		canTxData[2] = 0x00;
		canTxData[3] = 0x00;
		canTxData[4] = 0x00;
		canTxData[5] = 0x00;
		canTxData[6] = 0x00;
		canTxData[7] = 0x00;
		
		/* send can command */
		HAL_CAN_AddTxMessage(&hcan2, &canTxHeader, canTxData, &txMailBox);
	
}
void TurnOffMotor(uint8_t motorId)
{
		uint32_t txMailBox;
		canTxHeader.StdId = DEVICE_STD_ID + motorId;
		canTxHeader.ExtId = 0x00;
		canTxHeader.RTR = CAN_RTR_DATA;
		canTxHeader.IDE = CAN_ID_STD;
		canTxHeader.DLC = 8;
		
		canTxData[0] = 0x80;
		canTxData[1] = 0x00;
		canTxData[2] = 0x00;
		canTxData[3] = 0x00;
		canTxData[4] = 0x00;
		canTxData[5] = 0x00;
		canTxData[6] = 0x00;
		canTxData[7] = 0x00;
		
		/* send can command */
		HAL_CAN_AddTxMessage(&hcan2, &canTxHeader, canTxData, &txMailBox);
	
}
void MotorStop(uint8_t motorId)
{
		uint32_t txMailBox;
		canTxHeader.StdId = DEVICE_STD_ID + motorId;
		canTxHeader.ExtId = 0x00;
		canTxHeader.RTR = CAN_RTR_DATA;
		canTxHeader.IDE = CAN_ID_STD;
		canTxHeader.DLC = 8;
		
		canTxData[0] = 0x81;
		canTxData[1] = 0x00;
		canTxData[2] = 0x00;
		canTxData[3] = 0x00;
		canTxData[4] = 0x00;
		canTxData[5] = 0x00;
		canTxData[6] = 0x00;
		canTxData[7] = 0x00;
		
		/* send can command */
		HAL_CAN_AddTxMessage(&hcan2, &canTxHeader, canTxData, &txMailBox);
	
}
void MotorRun(uint8_t motorId)
{
		uint32_t txMailBox;
		canTxHeader.StdId = DEVICE_STD_ID + motorId;
		canTxHeader.ExtId = 0x00;
		canTxHeader.RTR = CAN_RTR_DATA;
		canTxHeader.IDE = CAN_ID_STD;
		canTxHeader.DLC = 8;
		
		canTxData[0] = 0x88;
		canTxData[1] = 0x00;
		canTxData[2] = 0x00;
		canTxData[3] = 0x00;
		canTxData[4] = 0x00;
		canTxData[5] = 0x00;
		canTxData[6] = 0x00;
		canTxData[7] = 0x00;
		
		/* send can command */
		HAL_CAN_AddTxMessage(&hcan2, &canTxHeader, canTxData, &txMailBox);
	
}
void TorqueOpenControlLoop(uint8_t motorId,int16_t powerControl)
{
		uint32_t txMailBox;
		canTxHeader.StdId = DEVICE_STD_ID + motorId;
		canTxHeader.ExtId = 0x00;
		canTxHeader.RTR = CAN_RTR_DATA;
		canTxHeader.IDE = CAN_ID_STD;
		canTxHeader.DLC = 8;
		
		canTxData[0] = 0xA0;
		canTxData[1] = 0x00;
		canTxData[2] = 0x00;
		canTxData[3] = 0x00;
		canTxData[4] = *((uint8_t *)&powerControl + 0);
		canTxData[5] = *((uint8_t *)&powerControl + 1);
		canTxData[6] = 0x00;
		canTxData[7] = 0x00;
		
		/* send can command */
		HAL_CAN_AddTxMessage(&hcan2, &canTxHeader, canTxData, &txMailBox);
	
}
void TorqueCloseControlLoop(uint8_t motorId,int16_t iqControl)
{
		uint32_t txMailBox;
		canTxHeader.StdId = DEVICE_STD_ID + motorId;
		canTxHeader.ExtId = 0x00;
		canTxHeader.RTR = CAN_RTR_DATA;
		canTxHeader.IDE = CAN_ID_STD;
		canTxHeader.DLC = 8;
		
		canTxData[0] = 0xA1;
		canTxData[1] = 0x00;
		canTxData[2] = 0x00;
		canTxData[3] = 0x00;
		canTxData[4] = *((uint8_t *)&iqControl + 0);
		canTxData[5] = *((uint8_t *)&iqControl + 1);
		canTxData[6] = 0x00;
		canTxData[7] = 0x00;
		
		/* send can command */
		HAL_CAN_AddTxMessage(&hcan2, &canTxHeader, canTxData, &txMailBox);

}
void SpeedCloseControlLoop(uint8_t motorId,int32_t speedControl)
{
		uint32_t txMailBox;
		canTxHeader.StdId = DEVICE_STD_ID + motorId;
		canTxHeader.ExtId = 0x00;
		canTxHeader.RTR = CAN_RTR_DATA;
		canTxHeader.IDE = CAN_ID_STD;
		canTxHeader.DLC = 8;
		
		canTxData[0] = 0xA2;
		canTxData[1] = 0x00;
		canTxData[2] = 0x00;
		canTxData[3] = *((uint8_t *)&speedControl + 0);
		canTxData[4] = *((uint8_t *)&speedControl + 1);
		canTxData[5] = *((uint8_t *)&speedControl + 2);
		canTxData[6] = *((uint8_t *)&speedControl + 3);
		canTxData[7] = 0x00;
		
		/* send can command */
		HAL_CAN_AddTxMessage(&hcan2, &canTxHeader, canTxData, &txMailBox);

}
/**
  * @function		AngleCloseControlLoop1
  * @brief          发送(电机id,控制速度)
  * @param[in]      motorId: 
  * @param[in]      angleControl: 电机位置多圈角度，方向由目标角度与实际角度的差值 ，范围[]
  * @param[in]      该控制模式下，电机的最大 加速度 由上位机(LK-Motor Tool)中Max Accleration值限制
  * @param[in]      该控制模式下，电机的最大 转矩   由上位机(LK-Motor Tool)中Max Torque Current值限制
  * @retval         36000代表360°
  * @retval         none
  */
void AngleCloseControlLoop1(uint8_t motorId,int32_t angleControl)
{
		uint32_t txMailBox;
		canTxHeader.StdId = DEVICE_STD_ID + motorId;
		canTxHeader.ExtId = 0x00;
		canTxHeader.RTR = CAN_RTR_DATA;
		canTxHeader.IDE = CAN_ID_STD;
		canTxHeader.DLC = 8;
		
		canTxData[0] = 0xA3;
		canTxData[1] = 0x00;
		canTxData[2] = 0x00;
		canTxData[3] = *((uint8_t *)&angleControl + 0);
		canTxData[4] = *((uint8_t *)&angleControl+ 1);
		canTxData[5] = *((uint8_t *)&angleControl + 2);
		canTxData[6] = *((uint8_t *)&angleControl + 3);
		canTxData[7] = 0x00;
		
		/* send can command */
		HAL_CAN_AddTxMessage(&hcan2, &canTxHeader, canTxData, &txMailBox);

}
/**
  * @function		AngleCloseControlLoop2
  * @brief          发送(电机id,控制角度，速度上限)
  * @param[in]      motorId: 
  * @param[in]      angleControl: 电机位置多圈角度，方向由目标角度与实际角度的差值 ，范围[]
  * @param[in]      maxSpeed:电机的最大 速度  限制
  * @param[in]      该控制模式下，电机的最大 角度  由上位机(LK-Motor Tool)中Max Angle值限制
  * @param[in]      该控制模式下，电机的最大 加速度 由上位机(LK-Motor Tool)中Max Accleration值限制
  * @param[in]      该控制模式下，电机的最大 转矩   由上位机(LK-Motor Tool)中Max Torque Current值限制
  * @retval         36000代表360°
  * @retval         none
  */
void AngleCloseControlLoop2(uint8_t motorId,int32_t angleControl,uint16_t maxSpeed)
{
		uint32_t txMailBox;
		canTxHeader.StdId = DEVICE_STD_ID + motorId;
		canTxHeader.ExtId = 0x00;
		canTxHeader.RTR = CAN_RTR_DATA;
		canTxHeader.IDE = CAN_ID_STD;
		canTxHeader.DLC = 8;
		
		canTxData[0] = 0xA4;
		canTxData[1] = 0x00;
		canTxData[2] = *((uint8_t *)&maxSpeed + 0);
		canTxData[3] = *((uint8_t *)&maxSpeed + 1);
		canTxData[4] = *((uint8_t *)&angleControl + 0);
		canTxData[5] = *((uint8_t *)&angleControl + 1);
		canTxData[6] = *((uint8_t *)&angleControl + 2);
		canTxData[7] = *((uint8_t *)&angleControl + 3);
		
		/* send can command */
		HAL_CAN_AddTxMessage(&hcan2, &canTxHeader, canTxData, &txMailBox);
}
/**
  * @function		AngleCloseControlLoop3
  * @brief          发送(电机id,控制角度，选择方向)
  * @param[in]      motorId: 
  * @param[in]      angleControl: 电机位置单圈角度，方向由符号决定 ，范围[0,359.99]
  * @param[in]      spinDirection:0x00->顺时针，0x01->逆时针
  * @param[in]      该控制模式下，电机的最大 加速度 由上位机(LK-Motor Tool)中Max Accleration值限制
  * @param[in]      该控制模式下，电机的最大 转矩   由上位机(LK-Motor Tool)中Max Torque Current值限制
  * @retval         none
  */
void AngleCloseControlLoop3(uint8_t motorId,uint16_t angleControl,uint8_t spinDirection)
{
		uint32_t txMailBox;
		canTxHeader.StdId = DEVICE_STD_ID + motorId;
		canTxHeader.ExtId = 0x00;
		canTxHeader.RTR = CAN_RTR_DATA;
		canTxHeader.IDE = CAN_ID_STD;
		canTxHeader.DLC = 8;
		
		canTxData[0] = 0xA5;
		canTxData[1] = spinDirection;
		canTxData[2] = 0x00;
		canTxData[3] = 0x00;
		canTxData[4] = *((uint8_t *)&angleControl + 0);
		canTxData[5] = *((uint8_t *)&angleControl + 1);
		canTxData[6] = 0x00;
		canTxData[7] = 0x00;
		
		/* send can command */
		HAL_CAN_AddTxMessage(&hcan2, &canTxHeader, canTxData, &txMailBox);

}
/**
  * @function		AngleCloseControlLoop4
  * @brief          发送(电机id,控制角度,旋转方向,最大速度)
  * @param[in]      motorId: 
  * @param[in]      angleControl: 电机位置单圈角度，方向由符号决定 ，范围[0,359.99]
  * @param[in]      spinDirection:0x00->顺时针，0x01->逆时针
  * @param[in]      maxSpeed:电机的最大 速度  限制
  * @retval         none
  */
void AngleCloseControlLoop4(uint8_t motorId,uint16_t angleControl,uint8_t spinDirection,uint16_t maxSpeed)
{
		uint32_t txMailBox;
		canTxHeader.StdId = DEVICE_STD_ID + motorId;
		canTxHeader.ExtId = 0x00;
		canTxHeader.RTR = CAN_RTR_DATA;
		canTxHeader.IDE = CAN_ID_STD;
		canTxHeader.DLC = 8;
		
		canTxData[0] = 0xA6;
		canTxData[1] = spinDirection;
		canTxData[2] = *((uint8_t *)&maxSpeed + 0);
		canTxData[3] = *((uint8_t *)&maxSpeed + 1);
		canTxData[4] = *((uint8_t *)&angleControl + 0);
		canTxData[5] = *((uint8_t *)&angleControl + 1);
		canTxData[6] = 0x00;
		canTxData[7] = 0x00;
		
		/* send can command */
		HAL_CAN_AddTxMessage(&hcan2, &canTxHeader, canTxData, &txMailBox);


}
/**
  * @function		AngleCloseControlLoop5
  * @brief          发送(电机id,控制角度)
  * @param[in]      motorId: 
  * @param[in]      angleControl: 电机位置增量控制，方向由符号决定 ，范围[]
  * @param[in]      该控制模式下，电机的最大 速度   由上位机(LK-Motor Tool)中Max Speed值限制
  * @param[in]      该控制模式下，电机的最大 加速度 由上位机(LK-Motor Tool)中Max Accleration值限制
  * @param[in]      该控制模式下，电机的最大 转矩   由上位机(LK-Motor Tool)中Max Torque Current值限制
  * @retval         none
  */
void AngleCloseControlLoop5(uint8_t motorId,int32_t angleControl)
{
		uint32_t txMailBox;
		canTxHeader.StdId = DEVICE_STD_ID + motorId;
		canTxHeader.ExtId = 0x00;
		canTxHeader.RTR = CAN_RTR_DATA;
		canTxHeader.IDE = CAN_ID_STD;
		canTxHeader.DLC = 8;
		
		canTxData[0] = 0xA7;
		canTxData[1] = 0x00;
		canTxData[2] = 0x00;
		canTxData[3] = 0x00;
		canTxData[4] = *((uint8_t *)&angleControl + 0);
		canTxData[5] = *((uint8_t *)&angleControl + 1);
		canTxData[6] = *((uint8_t *)&angleControl + 2);
		canTxData[7] = *((uint8_t *)&angleControl + 3);
		
		/* send can command */
		HAL_CAN_AddTxMessage(&hcan2, &canTxHeader, canTxData, &txMailBox);


}

/**
  * @function		AngleCloseControlLoop6
  * @brief          发送(电机id,控制角度,最大速度)
  * @param[in]      motorId: 
  * @param[in]      angleControl: 电机位置增量控制，方向由符号决定
  * @param[in]      maxSpeed: 范围 [0,]
  * @retval         none
  */
void AngleCloseControlLoop6(uint8_t motorId,int32_t angleControl,uint16_t maxSpeed)
{
		uint32_t txMailBox;
		canTxHeader.StdId = DEVICE_STD_ID + motorId;
		canTxHeader.ExtId = 0x00;
		canTxHeader.RTR = CAN_RTR_DATA;
		canTxHeader.IDE = CAN_ID_STD;
		canTxHeader.DLC = 8;
		
		canTxData[0] = 0xA8;
		canTxData[1] = 0x00;
		canTxData[2] = *((uint8_t *)&maxSpeed + 0);
		canTxData[3] = *((uint8_t *)&maxSpeed + 1);
		canTxData[4] = *((uint8_t *)&angleControl + 0);
		canTxData[5] = *((uint8_t *)&angleControl + 1);
		canTxData[6] = *((uint8_t *)&angleControl + 2);
		canTxData[7] = *((uint8_t *)&angleControl + 3);
		
		/* send can command */
		HAL_CAN_AddTxMessage(&hcan2, &canTxHeader, canTxData, &txMailBox);
}
