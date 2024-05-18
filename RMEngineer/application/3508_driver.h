/**
  ****************************(C) COPYRIGHT 2024 征途****************************
  * @file       3508_driver.c/h
  * @brief      
  *             这里是3508的驱动函数，通过pid控制控制电机.
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
	
	uint8_t motor_id;//电机id
	
	motor_measure_t basic_data;//电流，速度，温度
	
	
	pid_type_def  speed_control_data;//速度环pid相关参数
	
	pid_type_def  currrent_control_data;//电流环pid相关参数
	
	pid_type_def expecd_control_data;//编码器位置环pid相关参数
	
	struct{
		long long int exp_ecd;//绝对编码器值
		int exp_cycles;//旋转圈数
	}expand_data;
	
	long none;
	
}M3508_HandleTypeDef;

/**
  * @func			void M3508_Init(uint8_t motorid,M3508_HandleTypeDef* device,enum PID_MODE mode)
  * @brief          3508电机相关参数初始化
  * @param[in]      motorid:初始化电机id
  * @param[in]      device:3508电机句柄
  * @param[in]      pidmode:pid的控制模式(增量式，位置式)
  * @retval         none
  */
void M3508_GetBasicData(M3508_HandleTypeDef* device);

/**
  * @func			void M3508_GetBasicData(M3508_HandleTypeDef* device)
  * @brief          获取3508电机相关参数
  * @param[in]      device:3508电机句柄
  * @retval         none
  */
void M3508_Init(uint8_t motorid,M3508_HandleTypeDef* device,enum PID_MODE mode);

/**
  * @func			int16_t M3508_SpeedLoopPIDController(pid_type_def *pid, fp32 ref, fp32 set)
  * @brief          3508速度环pid控制函数
  * @param[in]      pid:3508电机速度环pid句柄
  * @param[in]      ref:3508电机实际转速
  * @param[in]      set:3508电机目标转速
  * @retval         3508速度环pid输出值
  */
int16_t M3508_SpeedLoopPIDController(pid_type_def *pid, fp32 ref, fp32 set);

/**
  * @func			int16_t M3508_CurrentLoopPIDController(pid_type_def *pid, fp32 ref, fp32 set)
  * @brief          3508电流环pid控制函数
  * @param[in]      pid:3508电机电流环pid句柄
  * @param[in]      ref:3508电机实际电流
  * @param[in]      set:3508电机目标电流
  * @retval         3508电流环pid输出值
  */
int16_t M3508_CurrentLoopPIDController(pid_type_def *pid, fp32 ref, fp32 set);

int16_t M3508_SpeedSyncPIDController(pid_type_def* pid ,fp32 ref,fp32 set);

void M3508_ExpandECD(M3508_HandleTypeDef* device);
int16_t M3508_ExpandEcdPIDController(pid_type_def* pid ,fp32 ref,fp32 set);
#endif

