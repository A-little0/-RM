/**
  ****************************(C) COPYRIGHT 2024 ��;****************************
  * @file       chassis_task.c/h
  * @brief      
  *             ��������̨������򣬰�����̨yaw���pitch��������Լ�pitch��������������
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Jan-29-2024    chenjiangnan    
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 ��;****************************
  */
#ifndef __GIMBAL_TASK_H
#define __GIMBAL_TASK_H

#include "3508_driver.h"
#include "arm_math.h"

/*���Ա���ֵת���������ı��� ��*/
#define TIFITING_ECD_TO_DISTANCE_CM 50
/*�������Ŀ����� ��*/
#define TIFITING_USING_POSITION_CONTROL 0
#define TIFITING_USING_SPEED_CONTROL 1
/*�����������pid��� ��*/
#define TIFITING_SPEEDSYNC_POSITIONGPID_KP 0
#define TIFITING_SPEEDSYNC_POSITIONGPID_KI 0
#define TIFITING_SPEEDSYNC_POSITIONGPID_KD 0

/*��̨�����ؾ��*/
extern M3508_HandleTypeDef lifiting_motor_left;
extern M3508_HandleTypeDef lifiting_motor_right;
extern M3508_HandleTypeDef drawer_motor_left;
extern M3508_HandleTypeDef drawer_motor_right;



/**
  * @func			void Gimbal_Init(void)
  * @brief          ��̨������ʼ��
  * @param[in]      none
  * @retval         none
  */
void Gimbal_Init(void);



void Gimbal_Task(int16_t updowmspeed,int16_t pushpullspeed);
/**
  * @func			void Printf_GimbalMessage(M6020_HandleTypeDef* device)
  * @brief          ��̨��������
  * @param[in]      targetangle������̨���Ŀ��λ��
  * @retval         none
  *void Printf_GimbalMessage(M6020_HandleTypeDef* device);
  */
#endif
