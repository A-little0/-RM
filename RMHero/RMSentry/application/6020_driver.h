/**
  ****************************(C) COPYRIGHT 2024 ��;****************************
  * @file       6020_driver.c/h
  * @brief      
  *             ������6020������������ͨ��pid���ƿ��Ƶ��.
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
#ifndef __6020_DRIVER_H
#define __6020_DRIVER_H

#include "main.h"
#include "CAN_receive.h"
#include "pid.h"
#include "user_lib.h"
/*YAW��pid������غ궨��*/		//������ϵͳ		//��������ϵͳ
#define YAWM6020_POSITIONPID_KP 0.25f//0.2f//0.25f//0.076f	//0.075f//0.07f//0.066f//0.065f
#define YAWM6020_POSITIONPID_KI 0.00025f//0.00025f//0.0f		//0.0015f//0.0013f//0.002f//0.002f
#define YAWM6020_POSITIONPID_KD 0.0f//0.5f//0.36f		//0.0f
#define YAWM6020_SPEEDPID_KP   220.0f//200.0f//600		//365.0f//360.0f//350.0f//300.0f
#define YAWM6020_SPEEDPID_KI  0.0f//0.0f
#define YAWM6020_SPEEDPID_KD  0.0f//0.0f

#define YAW_INTERGRAL_VALVE 60
/*PITCH��pid������غ궨��*/			//������ϵͳ		//��������ϵͳ
#define PITCHM6020_POSITIONPID_KP 0.85f//0.6f//0.4f		//0.15f
#define PITCHM6020_POSITIONPID_KI 0.0017f//0.01f 	//0.0001f
#define PITCHM6020_POSITIONPID_KD 0.0f//0.0f		//0.1f
#define PITCHM6020_SPEEDPID_KP 	 150.f//0.0f//310.0f		//30.0f
#define PITCHM6020_SPEEDPID_KI  0.0f//0.0f			//15.0f
#define PITCHM6020_SPEEDPID_KD  0.0f//0.0f          //10.0f

#define PITCH_INTERGRAL_VALVE 60					//100


enum CAN_CMD_MODE{
	 CAN_CMD_VOLTAGE=0xFF,
	 CAN_CMD_CURRENT=0xFE
};

typedef struct{
	
	uint8_t motor_id;//���id
	
	enum CAN_CMD_MODE cmd_mode;
	
	motor_measure_t basic_data;//�������ٶȣ��¶�
	
	pid_type_def  position_control_data;//λ�û�pid��ز���
	
	pid_type_def  speed_control_data;//�ٶȻ�pid��ز���
	
	pid_type_def  current_control_data;//��ѹ��pid��ز���
	
}M6020_HandleTypeDef;

/**
  * @func			void M6020_Init(uint8_t motorid,M6020_HandleTypeDef* device,enum PID_MODE pidmode,enum CAN_CMD_MODE cmdmode)
  * @brief          6020�����ز�����ʼ��
  * @param[in]      motorid:��ʼ�����id
  * @param[in]      device:6020������
  * @param[in]      pidmode:pid�Ŀ���ģʽ(����ʽ��λ��ʽ)
  * @param[in]      cmdmode:6020�Ŀ���ģʽ( CAN_CMD_VOLTAGE��can���͵�ѹ�����źţ�CAN_CMD_CURRENT��can���͵��������ź�)
  * @retval         none
  */
void M6020_Init(uint8_t motorid,M6020_HandleTypeDef* device,enum PID_MODE pidmode,enum CAN_CMD_MODE cmdmode);


/**
  * @func			void M6020_GetBasicData(M6020_HandleTypeDef* device)
  * @brief          ��ȡ6020�����ز���
  * @param[in]      device:6020������
  * @retval         none
  */
void M6020_GetBasicData(M6020_HandleTypeDef* device);


/**
  * @func			int16_t M6020_PositionLoopPIDController(pid_type_def *pid, fp32 ref, fp32 set)
  * @brief          6020λ�û�pid���ƺ���
  * @param[in]      pid:6020���λ�û�pid���
  * @param[in]      ref:6020���ʵ�ʽǶ�
  * @param[in]      set:6020���Ŀ��Ƕ�
  * @retval         6020λ�û�pid���ֵ
  */
int16_t M6020_PositionLoopPIDController(pid_type_def *pid, fp32 ref, fp32 set);

/**
  * @func			int16_t M6020_SpeedLoopPIDController(pid_type_def *pid, fp32 ref, fp32 set)
  * @brief          2006�ٶȻ�pid���ƺ���
  * @param[in]      pid:6020����ٶȻ�pid���
  * @param[in]      ref:6020���ʵ��ת��
  * @param[in]      set:6020���Ŀ��ת��
  * @retval         6020�ٶȻ�pid���ֵ
  */
int16_t M6020_SpeedLoopPIDController(pid_type_def *pid, fp32 ref, fp32 set);


/**
  * @func			int16_t M6020_CurrentLoopPIDController(pid_type_def *pid, fp32 ref, fp32 set)
  * @brief          6020������pid���ƺ���
  * @param[in]      pid:6020���������pid���
  * @param[in]      ref:6020���ʵ�ʵ���
  * @param[in]      set:6020���Ŀ�����
  * @retval         6020������pid���ֵ
  */
int16_t M6020_CurrentLoopPIDController(pid_type_def *pid, fp32 ref, fp32 set);

#endif
