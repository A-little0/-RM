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


typedef enum{
	
	//�����źŶ�ʧ
	RC_SIGNAL_UNLINK,

	//�Ƶ׿���ģʽ
	CHASSIS_CONTROL_DISABLE_MODE=1,
	
	CHASSIS_GYRO_MODE=3,
	
	CHASSIS_NORMAL_MODE=2,
	
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
  * @func			void Demo(void)
  * @brief          demo����
  * @param[in]      none
  * @retval         none
  */
void Demo(void);
#endif

