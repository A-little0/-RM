/**
  ****************************(C) COPYRIGHT 2024 征途****************************
  * @file       chassis_task.c/h
  * @brief      
  *             这里是云台任务程序，包含云台yaw轴和pitch轴的驱动以及pitch轴重力补偿程序
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
#ifndef __GIMBAL_TASK_H
#define __GIMBAL_TASK_H

#include "6020_driver.h"
#include "arm_math.h"
#include "jy901.h"



#define YAW_MEDIAN 4092//yaw 轴机械中值（编码器值）
//#define YAW_MEDIAN 156.917725f// yaw 轴机械中值

#define PITCH_MAXANGLE 4736//最大俯仰角
#define PITCH_MEDIAN 3405//pitch 轴机械中值（编码器）
#define PITCH_MINANGLE 2620//最小俯仰角

/*云台电机相关句柄*/
extern M6020_HandleTypeDef   yaw_gimbal_motor;
extern M6020_HandleTypeDef pitch_gimbal_motor;

/*重力补偿相关参数*/
extern int compensation_factor;
extern float pitch_cos;
extern float pitch_sin;
/*云台yaw轴相关参数*/
extern float yaw_beganangle;
extern int16_t yaw_beganecd;  //以编码器为参考
extern int yaw_key;
extern int yaw_key2;

/**
  * @func			void Gimbal_Init(void)
  * @brief          云台驱动初始化
  * @param[in]      none
  * @retval         none
  */
void Gimbal_Init(void);


/**
  * @func			int16_t Pitch_Gimbal_Task(int16_t targetpostion)
  * @brief          pitch轴串级pid控制任务
  * @param[in]      targetpostion：pitch轴目标绝对编码器值
  * @retval         pitch轴串级pid输出
  */
int16_t Pitch_Gimbal_Task(int16_t targetpostion);


/**
  * @func			int16_t GravityCompensation(const M6020_HandleTypeDef* device)
  * @brief          pitch轴重力补偿
  * @param[in]      device：6020电机句柄
  * @retval         补偿值
  */
int16_t GravityCompensation(const M6020_HandleTypeDef* device);



/**
  * @func			int16_t Yaw_Gimbal_Task(int16_t targetpostion)
  * @brief          yawh轴串级pid控制任务
  * @brief          以yawh轴绝对编码器为参考
  * @param[in]      targetpostion：yaw轴目标绝对编码器值
  * @retval         yaw轴串级pid输出
  */
int16_t Yaw_Gimbal_Task(int16_t targetpostion);



/**
  * @func			int16_t Yaw_Gimbal_Task2(float realangle ,float targetangle)
  * @brief          yawh轴串级pid控制任务2
  * @brief          以陀螺仪角度(世界坐标)为参考
  * @param[in]      realangle：yaw轴实时角度，范围[-180,180]
  * @param[in]      targetpostion：yaw轴目标角度,范围[-180,180]
  * @retval         yaw轴串级pid输出
  */
int16_t Yaw_Gimbal_Task2(float realangle ,float targetangle);


/**
  * @func			float Update_ReferenceBeganAngle(User_USART* jy901_data)
  * @brief          更新参考角度
  * @brief          以陀螺仪角度(世界坐标)为参考
  * @param[in]      jy901_data：jy901句柄
  * @retval         yaw轴更新后的参考角度
  */
float Update_ReferenceBeganAngle(User_USART* jy901_data);


/**
  * @func			float Get_ReferenceMedianAngle(User_USART* jy901_data,M6020_HandleTypeDef* device)
  * @brief          计算yaw轴在机械中值位置的角度
  * @brief          以陀螺仪角度(世界坐标)为参考
  * @param[in]      jy901_data：jy901句柄
  * @param[in]      device：6020电机句柄
  * @retval         yaw轴在机械中值位置的角度
  */
float Get_ReferenceMedianAngle(User_USART* jy901_data,M6020_HandleTypeDef* device);


/**
  * @func			int16_t Update_ReferenceBeganeECD(M6020_HandleTypeDef* device)
  * @brief          更新参考角度
  * @brief          以绝对编码值为参考
  * @param[in]      device：6020电机句柄
  * @retval         yaw轴更新后的参考角度(绝对编码器值)
  */
int16_t Update_ReferenceBeganeECD(M6020_HandleTypeDef* device);


/**
  * @func			void Printf_GimbalMessage(M6020_HandleTypeDef* device)
  * @brief          云台调试任务
  * @param[in]      targetangle：各云台电机目标位置
  * @retval         none
  */
void Printf_GimbalMessage(M6020_HandleTypeDef* device);

#endif
