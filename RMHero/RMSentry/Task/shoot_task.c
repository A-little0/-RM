/**
  ****************************(C) COPYRIGHT 2024 征途****************************
  * @file       shoot_task.c/h
  * @brief      
  *             这里是射击任务程序，包含摩擦轮驱动，拨盘驱动等功能程序
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Jan-29-2024    chenjiangnan    
  *
  @verbatim
  ==============================================================================
	bui~bui~bui~bui~bui~bui~bui~bui~bui~bui~bui~bui~bui~bui~bui~bui~bui~bui~bui
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 征途****************************
  */
#include "shoot_task.h"

/*拨盘电机句柄*/
M3508_HandleTypeDef trigger_motor;
/*摩擦轮电机句柄 */
M3508_HandleTypeDef left_frition_motor;
M3508_HandleTypeDef right_frition_motor;
/*射击模式相关参数*/
Shoot_ModeMessageTypedef shoot_modemessage;

/**
  * @func			void Shoot_Init(void)
  * @brief          射击驱动初始化
  * @param[in]      none
  * @retval         none
  */
void Shoot_Init(void)
{
	/*摩擦轮电机初始化*/
    M3508_Init(7,&left_frition_motor,PID_DELTA);
    M3508_Init(8,&right_frition_motor,PID_DELTA);
	/*拨弹电机初始化*/
    M3508_Init(5,&trigger_motor,PID_DELTA);
	/*拨弹信息初始化*/
	shoot_modemessage.valve=VALVE_CLOSE;
	shoot_modemessage.mode=-1;

}

//摩擦轮
int16_t SetFrition_Shoot_Task(M3508_HandleTypeDef* motor,int16_t targetspeed)
{
	int16_t intput=0;
	int16_t output=0;
	//获取电机信息
	M3508_GetBasicData(motor);
    //设置目标速度
	switch(0x200+motor->motor_id)
	{
		case CAN_FRITION_LEFT_MOTOR_ID:
			intput=targetspeed;
			break;
		case CAN_FRITION_RIGHT_MOTOR_ID:
			intput=-targetspeed;
			break;
		default:
			intput=0;
			break;
	}
	output=M3508_SpeedLoopPIDController(&(motor->speed_control_data),motor->basic_data.speed_rpm,intput);
	
	return output;
}
//拨弹
int16_t SetTrigger_Shoot_Task(M3508_HandleTypeDef* motor,int16_t targetspeed)
{
	int16_t output=M3508_SpeedLoopPIDController(&(motor->speed_control_data),motor->basic_data.speed_rpm,targetspeed);
	return output;
}











