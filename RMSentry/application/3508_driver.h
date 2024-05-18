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
/*�Ƿ�������ƽ���˲�*/
#define USING_SMOOTHING_FILTER 0// 1Ϊ����  0Ϊ�ر�

#if (USING_SMOOTHING_FILTER == 1)
	/*λ��PID*/							     //3
	#define M3508_ID1_SPEEDPID_POSITION_KP   10.0f
	#define M3508_ID1_SPEEDPID_POSITION_KI    0.1f
	#define M3508_ID1_SPEEDPID_POSITION_KD    0.0f

	#define M3508_ID2_SPEEDPID_POSITION_KP   13.5f
	#define M3508_ID2_SPEEDPID_POSITION_KI    1.1f
	#define M3508_ID2_SPEEDPID_POSITION_KD    0.0f

	#define M3508_ID3_SPEEDPID_POSITION_KP    10.0f
	#define M3508_ID3_SPEEDPID_POSITION_KI    0.1f
	#define M3508_ID3_SPEEDPID_POSITION_KD    0.0f

	#define M3508_ID4_SPEEDPID_POSITION_KP   10.0f
	#define M3508_ID4_SPEEDPID_POSITION_KI    0.1f
	#define M3508_ID4_SPEEDPID_POSITION_KD    0.0f
	
	/*����PID*/
	#define M3508_ID1_SPEEDPID_DELTA_KP   10.0f
	#define M3508_ID1_SPEEDPID_DELTA_KI    0.1f
	#define M3508_ID1_SPEEDPID_DELTA_KD    0.0f

	#define M3508_ID2_SPEEDPID_DELTA_KP   13.5f
	#define M3508_ID2_SPEEDPID_DELTA_KI    1.1f
	#define M3508_ID2_SPEEDPID_DELTA_KD    0.0f

	#define M3508_ID3_SPEEDPID_DELTA_KP    10.0f
	#define M3508_ID3_SPEEDPID_DELTA_KI    0.1f
	#define M3508_ID3_SPEEDPID_DELTA_KD    0.0f

	#define M3508_ID4_SPEEDPID_DELTA_KP   10.0f
	#define M3508_ID4_SPEEDPID_DELTA_KI    0.1f
	#define M3508_ID4_SPEEDPID_DELTA_KD    0.0f	
#else

	#define M3508_ID1_SPEEDPID_POSITION_KP   9.5f//19.0f
	#define M3508_ID1_SPEEDPID_POSITION_KI   0.1f//0.02f
	#define M3508_ID1_SPEEDPID_POSITION_KD   0.0f//0.0f

	#define M3508_ID2_SPEEDPID_POSITION_KP   9.5f//21.0f//17.0f//15.0f
	#define M3508_ID2_SPEEDPID_POSITION_KI   0.1f//0.1f//0.009f//0.008f
	#define M3508_ID2_SPEEDPID_POSITION_KD   0.0f// 0.0f

	#define M3508_ID3_SPEEDPID_POSITION_KP   20.0f//17.0f
	#define M3508_ID3_SPEEDPID_POSITION_KI   0.11f//0.008f
	#define M3508_ID3_SPEEDPID_POSITION_KD   0.0f//0.0f

	#define M3508_ID4_SPEEDPID_POSITION_KP   10.0f//20.0f//19.0f
	#define M3508_ID4_SPEEDPID_POSITION_KI   0.1f//0.02f
	#define M3508_ID4_SPEEDPID_POSITION_KD   0.0f//0.0f
	
	/*����PID*/
	#define M3508_ID1_SPEEDPID_DELTA_KP    9.5f
	#define M3508_ID1_SPEEDPID_DELTA_KI    0.1f
	#define M3508_ID1_SPEEDPID_DELTA_KD    0.0f

	#define M3508_ID2_SPEEDPID_DELTA_KP    15.0f
	#define M3508_ID2_SPEEDPID_DELTA_KI    0.2f
	#define M3508_ID2_SPEEDPID_DELTA_KD    0.0f

	#define M3508_ID3_SPEEDPID_DELTA_KP    20.0f
	#define M3508_ID3_SPEEDPID_DELTA_KI    0.1f
	#define M3508_ID3_SPEEDPID_DELTA_KD    0.0f

	#define M3508_ID4_SPEEDPID_DELTA_KP    15.0f
	#define M3508_ID4_SPEEDPID_DELTA_KI    0.1f
	#define M3508_ID4_SPEEDPID_DELTA_KD    0.0f	

#endif

#if (USING_SMOOTHING_FILTER == 1)
	#define M3508_ID1_DATALENGTH 3
	#define M3508_ID2_DATALENGTH 3
	#define M3508_ID3_DATALENGTH 3
	#define M3508_ID4_DATALENGTH 3
#endif
typedef struct{
	
	uint8_t motor_id;//���id
	
	motor_measure_t basic_data;//�������ٶȣ��¶�
	
	pid_type_def  speed_control_data;//�ٶȻ�pid��ز���
	
	pid_type_def  currrent_control_data;//������pid��ز���
	
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
#endif

