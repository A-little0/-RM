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
#include "shoot_task.h"

typedef enum{
	
	//控制信号丢失
	RC_SIGNAL_UNLINK,

	//云底控制模式
	CHASSIS_NORMAL_MODE=1,
	
	CHASSIS_GYRO_MODE=2,
	
	CHASSIS_FOLLOW_GIMBAL_MODE=3,
	
	AUTO_AIM_MODE=4,
	
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
  * @func			void RC_Process(void)
  * @brief          处理控制数据
  * @param[in]      none
  * @retval         none
  */
void RC_Process(void);

/**
  * @func			void RC_Remote_Process(int mode)
  * @brief          处理控制数据
  * @param[in]      none
  * @retval         none
  */
void RC_Remote_Process(int mode);

/**
  * @func			void Keyboard_Process(int mode)
  
  * @brief          处理控制数据
  * @param[in]      none
  * @retval         none
  */
void Keyboard_Process(int mode);
#endif

