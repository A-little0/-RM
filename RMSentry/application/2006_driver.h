/**
  ****************************(C) COPYRIGHT 2024 ��;****************************
  * @file       2006_driver.c/h
  * @brief      
  *             ������2006������������ͨ��pid���ƿ��Ƶ��.
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
#ifndef __2006_DRIVER_H
#define __2006_DRIVER_H

#include "main.h"
#include "CAN_receive.h"
#include "pid.h"
#include "user_lib.h"

#define M2006_REDUCTION_RATIO 11.000000 //���ٱ�:36:1
//#define M2006_RECORD_EXP_ECDBOSX_LENGTH 10

/*λ��PID����*/
#define UP2006_SPEEDPID_POSITION_KP 10.0f
#define UP2006_SPEEDPID_POSITION_KI 0.0f
#define UP2006_SPEEDPID_POSITION_KD 0.0f
#define DOWN2006_SPEEDPID_POSITION_KP 10.0f
#define DOWN2006_SPEEDPID_POSITION_KI 0.0f
#define DOWN2006_SPEEDPID_POSITION_KD 0.0f

/*����ʽPID����*/
#define UP2006_SPEEDPID_DELTA_KP 0.0f
#define UP2006_SPEEDPID_DELTA_KI 0.0f
#define UP2006_SPEEDPID_DELTA_KD 0.0f
#define DOWN2006_SPEEDPID_DELTA_KP 0.0f
#define DOWN2006_SPEEDPID_DELTA_KI 0.0f
#define DOWN2006_SPEEDPID_DELTA_KD 0.0f

typedef struct{
	
	uint8_t motor_id;//���id
	
	int  expand_ecd;//�����������ת�� ���Ա�����ֵ
	
	int  cylinder_number;//��תȦ��
	
	int16_t record_last_ecd;//���Ϊת�ӵ���һ�εľ��Ա�����ֵ
	
//	int16_t record_exp_ecdboxs[M2006_RECORD_EXP_ECDBOSX_LENGTH];//��¼ת�Ӿ��Ա�����ֵ
	
	motor_measure_t basic_data;//�������ٶȣ��¶�
	
    pid_type_def  position_control_data;//λ�û�pid��ز���
	
	pid_type_def  speed_control_data;//�ٶȻ�pid��ز���
	
	pid_type_def  currrent_control_data;//������pid��ز���
	
}M2006_HandleTypeDef;


//extern M2006_HandleTypeDef up_trigger_motor;
//extern M2006_HandleTypeDef down_trigger_motor;

/**
  * @func			void M2006_Init(uint8_t motorid,M2006_HandleTypeDef* device,enum PID_MODE pidmode)
  * @brief          2006�����ز�����ʼ��
  * @param[in]      motorid:��ʼ�����id
  * @param[in]      device:2006������
  * @param[in]      pidmode:pid�Ŀ���ģʽ(����ʽ��λ��ʽ)
  * @retval         none
  */
void M2006_Init(uint8_t motorid,M2006_HandleTypeDef* device,enum PID_MODE pidmode);


/**
  * @func			void M2006_GetBasicData(M2006_HandleTypeDef* device)
  * @brief          ��ȡ2006�����ز���
  * @param[in]      device:2006������
  * @retval         none
  */
void M2006_GetBasicData(M2006_HandleTypeDef* device);


/**
  * @func			int16_t M2006_GetExpandECDData(M2006_HandleTypeDef* device)
  * @brief          ��ȡ2006������������ԽǶ�(�Ա������Ƕȷ�Χ����������)
  * @brief          �ú����Ե�����ת�ĵ����Ч��������ת���ܻ���ڶ�Ȧ����
  * @param[in]      device:2006������
  * @retval         �����ı������Ƕ�ֵ
  */
int16_t M2006_GetExpandECDData(M2006_HandleTypeDef* device);


/**
  * @func			float M2006_AbsoluteEncoderToAngle(int absoluteencoder)
  * @brief          ��2006���(ת��)���������ԽǶ�ֵ ת��Ϊ (������)���������ԽǶ�ֵ
  * @param[in]      absoluteencoder:2006���(ת��)���������ԽǶ�ֵ
  * @retval         (������)���������ԽǶ�ֵ
  */
float M2006_AbsoluteEncoderToAngle(int absoluteencoder);

/**
  * @func			_userbool M2006_LockedMotorDetertion(M2006_HandleTypeDef* device)
  * @brief          2006�����ת���
  * @param[in]      device:2006������
  * @retval         boolֵ��user_trueΪ�����ת������Ϊ���δ��ת
  */
_userbool M2006_LockedMotorDetertion(M2006_HandleTypeDef* device);
  
  
 /**
  * @func			int16_t M2006_PositionLoopPIDController(pid_type_def *pid, fp32 ref, fp32 set)
  * @brief          2006λ�û�pid���ƺ���
  * @param[in]      pid:2006���λ�û�pid���
  * @param[in]      ref:2006���ʵ�ʽǶ�
  * @param[in]      set:2006���Ŀ��Ƕ�
  * @retval         2006λ�û�pid���ֵ
  */
int16_t M2006_PositionLoopPIDController(pid_type_def *pid, fp32 ref, fp32 set);


/**
  * @func			int16_t M2006_SpeedLoopPIDController(pid_type_def *pid, fp32 ref, fp32 set)
  * @brief          2006�ٶȻ�pid���ƺ���
  * @param[in]      pid:2006����ٶȻ�pid���
  * @param[in]      ref:2006���ʵ��ת��
  * @param[in]      set:2006���Ŀ��ת��
  * @retval         2006�ٶȻ�pid���ֵ
  */
int16_t M2006_SpeedLoopPIDController(pid_type_def *pid, fp32 ref, fp32 set);


/**
  * @func			int16_t M2006_CurrentLoopPIDController(pid_type_def *pid, fp32 ref, fp32 set)
  * @brief          2006������pid���ƺ���
  * @param[in]      pid:2006���������pid���
  * @param[in]      ref:2006���ʵ�ʵ���
  * @param[in]      set:2006���Ŀ�����
  * @retval         2006������pid���ֵ
  */
int16_t M2006_CurrentLoopPIDController(pid_type_def *pid, fp32 ref, fp32 set);
#endif
