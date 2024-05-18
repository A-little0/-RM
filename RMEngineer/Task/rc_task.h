/**
  ****************************(C) COPYRIGHT 2024 征途****************************
  * @file       rc_task.c/h
  * @brief      
  *             这里是demo任务程序,嗯~
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Jan-29-2024    chenjiangnan      有待提高
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 征途****************************
  */
#ifndef __RC_TASK_H
#define __RC_TASK_H

#include "rc_receive.h"
#include "chassis_task.h"
#include "gimbal_task.h"


typedef enum{
	
	//控制信号丢失
	RC_SIGNAL_UNLINK,

	//云底控制模式
	CHASSIS_CONTROL_DISABLE_MODE=1,
	
	CHASSIS_GYRO_MODE=3,
	
	CHASSIS_NORMAL_MODE=2,
	
}RC_ControlMode;

/*遥控器模式句柄*/
extern RC_ControlMode rc_control_mode;


/**
  * @func			void RC_TaskInit(void)
  * @brief          遥控器任务初始化
  * @param[in]      none
  * @retval         none
  */
void RC_TaskInit(void);


/**
  * @func			void Demo(void)
  * @brief          demo程序
  * @param[in]      none
  * @retval         none
  */
void Demo(void);
#endif

