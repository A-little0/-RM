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
/*是否开启滑动平均滤波*/
#define USING_SMOOTHING_FILTER 0// 1为开启  0为关闭

#if (USING_SMOOTHING_FILTER == 1)
	/*位置PID*/							     //3
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
	
	/*增量PID*/
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
	
	/*增量PID*/
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
	
	uint8_t motor_id;//电机id
	
	motor_measure_t basic_data;//电流，速度，温度
	
	pid_type_def  speed_control_data;//速度环pid相关参数
	
	pid_type_def  currrent_control_data;//电流环pid相关参数
	
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
#endif

