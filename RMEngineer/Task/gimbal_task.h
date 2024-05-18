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
#ifndef __GIMBAL_TASK_H
#define __GIMBAL_TASK_H

#include "3508_driver.h"
#include "arm_math.h"

/*绝对编码值转升降机构的比例 宏*/
#define TIFITING_ECD_TO_DISTANCE_CM 50
/*升降电机目标控制 宏*/
#define TIFITING_USING_POSITION_CONTROL 0
#define TIFITING_USING_SPEED_CONTROL 1
/*升降机构电机pid相关 宏*/
#define TIFITING_SPEEDSYNC_POSITIONGPID_KP 0
#define TIFITING_SPEEDSYNC_POSITIONGPID_KI 0
#define TIFITING_SPEEDSYNC_POSITIONGPID_KD 0

/*云台电机相关句柄*/
extern M3508_HandleTypeDef lifiting_motor_left;
extern M3508_HandleTypeDef lifiting_motor_right;
extern M3508_HandleTypeDef drawer_motor_left;
extern M3508_HandleTypeDef drawer_motor_right;



/**
  * @func			void Gimbal_Init(void)
  * @brief          云台驱动初始化
  * @param[in]      none
  * @retval         none
  */
void Gimbal_Init(void);



void Gimbal_Task(int16_t updowmspeed,int16_t pushpullspeed);
/**
  * @func			void Printf_GimbalMessage(M6020_HandleTypeDef* device)
  * @brief          云台调试任务
  * @param[in]      targetangle：各云台电机目标位置
  * @retval         none
  *void Printf_GimbalMessage(M6020_HandleTypeDef* device);
  */
#endif
