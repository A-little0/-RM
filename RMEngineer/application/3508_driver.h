/**
  ****************************(C) COPYRIGHT 2024 ��;****************************
  * @file       3508_driver.c/h
  * @brief      
  *             ������3508������������ͨ��pid���ƿ��Ƶ��.
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
#ifndef __3508_DRIVER_H
#define __3508_DRIVER_H

#include "main.h"
#include "pid.h"
#include "CAN_receive.h"

#define M3508_ID1_SPEEDPID_POSITION_KP   11.0f
#define M3508_ID1_SPEEDPID_POSITION_KI    0.1f
#define M3508_ID1_SPEEDPID_POSITION_KD   0.0f

#define M3508_ID2_SPEEDPID_POSITION_KP  11.0f
#define M3508_ID2_SPEEDPID_POSITION_KI   0.1f
#define M3508_ID2_SPEEDPID_POSITION_KD   0.0f

#define M3508_ID3_SPEEDPID_POSITION_KP   12.0f
#define M3508_ID3_SPEEDPID_POSITION_KI    0.1f
#define M3508_ID3_SPEEDPID_POSITION_KD    0.0f

#define M3508_ID4_SPEEDPID_POSITION_KP   11.0f
#define M3508_ID4_SPEEDPID_POSITION_KI   0.05f
#define M3508_ID4_SPEEDPID_POSITION_KD    0.0f

/******************************************************/

#define LIFITING_LEFT_SPEEDPID_POSITION_KP  20.0f
#define LIFITING_LEFT_SPEEDPID_POSITION_KI  0.0f
#define LIFITING_LEFT_SPEEDPID_POSITION_KD  0.0f

#define LIFITING_RIGHT_SPEEDPID_POSITION_KP  20.0f
#define LIFITING_RIGHT_SPEEDPID_POSITION_KI  0.0f
#define LIFITING_RIGHT_SPEEDPID_POSITION_KD  0.0f

#define DRAWER_LEFT_SPEEDPID_POSITION_KP    20.0f
#define DRAWER_LEFT_SPEEDPID_POSITION_KI    0.0f
#define DRAWER_LEFT_SPEEDPID_POSITION_KD    0.0f

#define DRAWER_RIGHT_SPEEDPID_POSITION_KP    20.0f
#define DRAWER_RIGHT_SPEEDPID_POSITION_KI    0.0f
#define DRAWER_RIGHT_SPEEDPID_POSITION_KD    0.0f

/********************************************************/
#define LIFITING_LEFT_EXPECDPID_POSITION_KP  0.0f
#define LIFITING_LEFT_EXPECDPID_POSITION_KI  0.0f
#define LIFITING_LEFT_EXPECDPID_POSITION_KD  0.0f

#define LIFITING_RIGHT_EXPECDPID_POSITION_KP  0.0f
#define LIFITING_RIGHT_EXPECDPID_POSITION_KI  0.0f
#define LIFITING_RIGHT_EXPECDPID_POSITION_KD  0.0f

#define DRAWER_LEFT_EXPECDPID_POSITION_KP    0.0f
#define DRAWER_LEFT_EXPECDPID_POSITION_KI    0.009f
#define DRAWER_LEFT_EXPECDPID_POSITION_KD    0.0f

#define DRAWER_RIGHT_EXPECDPID_POSITION_KP    0.0f
#define DRAWER_RIGHT_EXPECDPID_POSITION_KI    0.0f
#define DRAWER_RIGHT_EXPECDPID_POSITION_KD    0.0f





typedef struct{
	
	uint8_t motor_id;//���id
	
	motor_measure_t basic_data;//�������ٶȣ��¶�
	
	
	pid_type_def  speed_control_data;//�ٶȻ�pid��ز���
	
	pid_type_def  currrent_control_data;//������pid��ز���
	
	pid_type_def expecd_control_data;//������λ�û�pid��ز���
	
	struct{
		long long int exp_ecd;//���Ա�����ֵ
		int exp_cycles;//��תȦ��
	}expand_data;
	
	long none;
	
}M3508_HandleTypeDef;

/**
  * @func			void M3508_Init(uint8_t motorid,M3508_HandleTypeDef* device,enum PID_MODE mode)
  * @brief          3508�����ز�����ʼ��
  * @param[in]      motorid:��ʼ�����id
  * @param[in]      device:3508������
  * @param[in]      pidmode:pid�Ŀ���ģʽ(����ʽ��λ��ʽ)
  * @retval         none
  */
void M3508_GetBasicData(M3508_HandleTypeDef* device);

/**
  * @func			void M3508_GetBasicData(M3508_HandleTypeDef* device)
  * @brief          ��ȡ3508�����ز���
  * @param[in]      device:3508������
  * @retval         none
  */
void M3508_Init(uint8_t motorid,M3508_HandleTypeDef* device,enum PID_MODE mode);

/**
  * @func			int16_t M3508_SpeedLoopPIDController(pid_type_def *pid, fp32 ref, fp32 set)
  * @brief          3508�ٶȻ�pid���ƺ���
  * @param[in]      pid:3508����ٶȻ�pid���
  * @param[in]      ref:3508���ʵ��ת��
  * @param[in]      set:3508���Ŀ��ת��
  * @retval         3508�ٶȻ�pid���ֵ
  */
int16_t M3508_SpeedLoopPIDController(pid_type_def *pid, fp32 ref, fp32 set);

/**
  * @func			int16_t M3508_CurrentLoopPIDController(pid_type_def *pid, fp32 ref, fp32 set)
  * @brief          3508������pid���ƺ���
  * @param[in]      pid:3508���������pid���
  * @param[in]      ref:3508���ʵ�ʵ���
  * @param[in]      set:3508���Ŀ�����
  * @retval         3508������pid���ֵ
  */
int16_t M3508_CurrentLoopPIDController(pid_type_def *pid, fp32 ref, fp32 set);

int16_t M3508_SpeedSyncPIDController(pid_type_def* pid ,fp32 ref,fp32 set);

void M3508_ExpandECD(M3508_HandleTypeDef* device);
int16_t M3508_ExpandEcdPIDController(pid_type_def* pid ,fp32 ref,fp32 set);
#endif

