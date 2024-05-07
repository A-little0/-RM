/**
  ****************************(C) COPYRIGHT 2024 ��;****************************
  * @file       rc_task.c/h
  * @brief      
  *             ������demo�������,��~
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Jan-29-2024    chenjiangnan      �д����
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 ��;****************************
  */
#ifndef __RC_TASK_H
#define __RC_TASK_H

#include "rc_receive.h"
#include "chassis_task.h"
#include "gimbal_task.h"
#include "shoot_task.h"

typedef enum{
	
	//�����źŶ�ʧ
	RC_SIGNAL_UNLINK,

	//�Ƶ׿���ģʽ
	CHASSIS_NORMAL_MODE=1,
	
	CHASSIS_GYRO_MODE=2,
	
	CHASSIS_FOLLOW_GIMBAL_MODE=3,
	
	AUTO_AIM_MODE=4,
	
}RC_ControlMode;

/*ң����ģʽ���*/
extern RC_ControlMode rc_control_mode;


/**
  * @func			void RC_TaskInit(void)
  * @brief          ң���������ʼ��
  * @param[in]      none
  * @retval         none
  */
void RC_TaskInit(void);

/**
  * @func			void RC_Process(void)
  * @brief          �����������
  * @param[in]      none
  * @retval         none
  */
void RC_Process(void);

/**
  * @func			void RC_Remote_Process(int mode)
  * @brief          �����������
  * @param[in]      none
  * @retval         none
  */
void RC_Remote_Process(int mode);

/**
  * @func			void Keyboard_Process(int mode)
  
  * @brief          �����������
  * @param[in]      none
  * @retval         none
  */
void Keyboard_Process(int mode);
#endif

