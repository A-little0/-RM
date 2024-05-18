/**
  ****************************(C) COPYRIGHT 2024 征途****************************
  * @file       chassis_task.c/h
  * @brief      
  *             这里是云台任务程序，包含云台yaw轴和pitch轴的驱动以及pitch轴重力补偿程序
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Jan-29-2024    chenjiangnan    
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 征途****************************
  */
#include "gimbal_task.h"

/*云台电机相关句柄*/
M3508_HandleTypeDef lifiting_motor_left;
M3508_HandleTypeDef lifiting_motor_right;
M3508_HandleTypeDef drawer_motor_left;
M3508_HandleTypeDef drawer_motor_right;
/*抬升电机同步pid*/
pid_type_def gimbal_lifiting_speedsync_pid;
int16_t errorspeed=0;
/************/
float lifiting_hight_cm;//抬升高度(单位：cm)
float drawer_length_cm;//伸缩长度(单位:cm)



/**
  * @func			void Gimbal_Init(void)
  * @brief          云台驱动初始化
  * @param[in]      none
  * @retval         none
  */
void Gimbal_Init(void)
{
	/*初始化云台相关电机*/
	M3508_Init(5,&lifiting_motor_left, PID_POSITION);
	M3508_Init(6,&lifiting_motor_right,PID_POSITION);
	M3508_Init(7,&drawer_motor_left,PID_POSITION);
	M3508_Init(8,&drawer_motor_right,PID_POSITION);
	/*初始化抬升电机同步pid*/
	float speedsync_pid[3]={
		TIFITING_SPEEDSYNC_POSITIONGPID_KP,
		TIFITING_SPEEDSYNC_POSITIONGPID_KI,
		TIFITING_SPEEDSYNC_POSITIONGPID_KD
	};
	
	PID_init(&gimbal_lifiting_speedsync_pid,PID_POSITION,speedsync_pid,16384,16384);
}

void Gimbal_Task(int16_t updowmspeed,int16_t pushpullspeed)
{
    /*获取云台电机基础数据*/
	M3508_GetBasicData(&lifiting_motor_left);
	M3508_GetBasicData(&lifiting_motor_right);
	M3508_GetBasicData(&drawer_motor_left);
	M3508_GetBasicData(&drawer_motor_right);
	/*获取目标值*/
	int16_t targetspeedMA= updowmspeed;
	int16_t targetspeedMB= updowmspeed;
	int16_t targetspeedMC= pushpullspeed;
	int16_t targetspeedMD= pushpullspeed;
	/*速度pid运算*/
	int16_t set_currentMA=M3508_SpeedLoopPIDController(
		&lifiting_motor_left.speed_control_data,
		lifiting_motor_left.basic_data.speed_rpm, 
		targetspeedMA
	);
	int16_t set_currentMB=M3508_SpeedLoopPIDController(
		&lifiting_motor_right.speed_control_data, 
		lifiting_motor_right.basic_data.speed_rpm, 
		targetspeedMB
	);
	int16_t set_currentMC=M3508_SpeedLoopPIDController(
		&drawer_motor_left.speed_control_data, 
		drawer_motor_left.basic_data.speed_rpm, 
		targetspeedMC
	);
	int16_t set_currentMD=M3508_SpeedLoopPIDController(
		&drawer_motor_right.speed_control_data,
		drawer_motor_right.basic_data.speed_rpm, 
		targetspeedMD
	);
	/*升降电机速度同步pid  交叉耦合*/
	errorspeed =lifiting_motor_left.basic_data.speed_rpm + lifiting_motor_right.basic_data.speed_rpm;
	int16_t syncoutputMA=-M3508_SpeedSyncPIDController(&gimbal_lifiting_speedsync_pid,errorspeed,0);
	int16_t syncoutputMB=-syncoutputMA;
	/*升降电机速度同步pid  主从控制*/
	
	
	/*发送电流至云台电机*/
	//CAN_cmd_gimbal(set_currentMA,0,0,0);
	CAN_cmd_gimbal(set_currentMA,set_currentMB,set_currentMC,set_currentMD);
	//CAN_cmd_gimbal(set_currentMA+syncoutputMA,set_currentMB+syncoutputMB,set_currentMC,set_currentMD);
}

/**
  * @func			void Printf_GimbalMessage(M6020_HandleTypeDef* device)
  * @brief          云台调试任务
  * @param[in]      targetangle：各云台电机目标位置
  * @retval         none
  */
void Printf_GimbalMessage()
{
	
}
