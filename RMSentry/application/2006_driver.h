/**
  ****************************(C) COPYRIGHT 2024 征途****************************
  * @file       2006_driver.c/h
  * @brief      
  *             这里是2006的驱动函数，通过pid控制控制电机.
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
#ifndef __2006_DRIVER_H
#define __2006_DRIVER_H

#include "main.h"
#include "CAN_receive.h"
#include "pid.h"
#include "user_lib.h"

#define M2006_REDUCTION_RATIO 11.000000 //减速比:36:1
//#define M2006_RECORD_EXP_ECDBOSX_LENGTH 10

/*位置PID参数*/
#define UP2006_SPEEDPID_POSITION_KP 10.0f
#define UP2006_SPEEDPID_POSITION_KI 0.0f
#define UP2006_SPEEDPID_POSITION_KD 0.0f
#define DOWN2006_SPEEDPID_POSITION_KP 10.0f
#define DOWN2006_SPEEDPID_POSITION_KI 0.0f
#define DOWN2006_SPEEDPID_POSITION_KD 0.0f

/*增量式PID参数*/
#define UP2006_SPEEDPID_DELTA_KP 0.0f
#define UP2006_SPEEDPID_DELTA_KI 0.0f
#define UP2006_SPEEDPID_DELTA_KD 0.0f
#define DOWN2006_SPEEDPID_DELTA_KP 0.0f
#define DOWN2006_SPEEDPID_DELTA_KI 0.0f
#define DOWN2006_SPEEDPID_DELTA_KD 0.0f

typedef struct{
	
	uint8_t motor_id;//电机id
	
	int  expand_ecd;//电机减速箱旋转轴 绝对编码器值
	
	int  cylinder_number;//旋转圈数
	
	int16_t record_last_ecd;//这个为转子的上一次的绝对编码器值
	
//	int16_t record_exp_ecdboxs[M2006_RECORD_EXP_ECDBOSX_LENGTH];//记录转子绝对编码器值
	
	motor_measure_t basic_data;//电流，速度，温度
	
    pid_type_def  position_control_data;//位置环pid相关参数
	
	pid_type_def  speed_control_data;//速度环pid相关参数
	
	pid_type_def  currrent_control_data;//电流环pid相关参数
	
}M2006_HandleTypeDef;


//extern M2006_HandleTypeDef up_trigger_motor;
//extern M2006_HandleTypeDef down_trigger_motor;

/**
  * @func			void M2006_Init(uint8_t motorid,M2006_HandleTypeDef* device,enum PID_MODE pidmode)
  * @brief          2006电机相关参数初始化
  * @param[in]      motorid:初始化电机id
  * @param[in]      device:2006电机句柄
  * @param[in]      pidmode:pid的控制模式(增量式，位置式)
  * @retval         none
  */
void M2006_Init(uint8_t motorid,M2006_HandleTypeDef* device,enum PID_MODE pidmode);


/**
  * @func			void M2006_GetBasicData(M2006_HandleTypeDef* device)
  * @brief          获取2006电机相关参数
  * @param[in]      device:2006电机句柄
  * @retval         none
  */
void M2006_GetBasicData(M2006_HandleTypeDef* device);


/**
  * @func			int16_t M2006_GetExpandECDData(M2006_HandleTypeDef* device)
  * @brief          获取2006电机编码器绝对角度(对编码器角度范围进行了扩充)
  * @brief          该函数对低速旋转的电机有效，高速旋转可能会存在丢圈问题
  * @param[in]      device:2006电机句柄
  * @retval         扩充后的编码器角度值
  */
int16_t M2006_GetExpandECDData(M2006_HandleTypeDef* device);


/**
  * @func			float M2006_AbsoluteEncoderToAngle(int absoluteencoder)
  * @brief          将2006电机(转子)编码器绝对角度值 转换为 (减速箱)编码器绝对角度值
  * @param[in]      absoluteencoder:2006电机(转子)编码器绝对角度值
  * @retval         (减速箱)编码器绝对角度值
  */
float M2006_AbsoluteEncoderToAngle(int absoluteencoder);

/**
  * @func			_userbool M2006_LockedMotorDetertion(M2006_HandleTypeDef* device)
  * @brief          2006电机堵转检测
  * @param[in]      device:2006电机句柄
  * @retval         bool值，user_true为电机堵转，否则为电机未堵转
  */
_userbool M2006_LockedMotorDetertion(M2006_HandleTypeDef* device);
  
  
 /**
  * @func			int16_t M2006_PositionLoopPIDController(pid_type_def *pid, fp32 ref, fp32 set)
  * @brief          2006位置环pid控制函数
  * @param[in]      pid:2006电机位置环pid句柄
  * @param[in]      ref:2006电机实际角度
  * @param[in]      set:2006电机目标角度
  * @retval         2006位置环pid输出值
  */
int16_t M2006_PositionLoopPIDController(pid_type_def *pid, fp32 ref, fp32 set);


/**
  * @func			int16_t M2006_SpeedLoopPIDController(pid_type_def *pid, fp32 ref, fp32 set)
  * @brief          2006速度环pid控制函数
  * @param[in]      pid:2006电机速度环pid句柄
  * @param[in]      ref:2006电机实际转速
  * @param[in]      set:2006电机目标转速
  * @retval         2006速度环pid输出值
  */
int16_t M2006_SpeedLoopPIDController(pid_type_def *pid, fp32 ref, fp32 set);


/**
  * @func			int16_t M2006_CurrentLoopPIDController(pid_type_def *pid, fp32 ref, fp32 set)
  * @brief          2006电流环pid控制函数
  * @param[in]      pid:2006电机电流环pid句柄
  * @param[in]      ref:2006电机实际电流
  * @param[in]      set:2006电机目标电流
  * @retval         2006电流环pid输出值
  */
int16_t M2006_CurrentLoopPIDController(pid_type_def *pid, fp32 ref, fp32 set);
#endif
