/**
  ****************************(C) COPYRIGHT 2024 征途****************************
  * @file       chassis_task.c/h
  * @brief      
  *             这里是麦轮底盘任务程序，包含麦轮解算，底盘小陀螺，底盘跟随云台功能程序
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
#ifndef __CHASSIS_TASK_H
#define __CHASSIS_TASK_H

#include "3508_driver.h"
#include "arm_math.h"
#include "user_lib.h"
/**/

/*小陀螺旋转速度*/
#define CHASSIS_GYRO_ROTATE_SPEED 3000
#define CHASSIS_MAX_XVECTOR_SPEED 2500
#define CHASSIS_MAX_YVECTOR_SPEED 2500
/*底盘pid相关宏*/
#define CHASSIS_ANGLEPID_KP 1.5f 
#define CHASSIS_ANGLEPID_KI 0.0f 
#define CHASSIS_ANGLEPID_KD 0.0f 

typedef struct{
	//底盘建系
	int16_t c_x;//指向底盘前方
	int16_t c_y;//指向底盘底盘前方左侧
	//底盘映射-》世界
	int16_t c_rotate;//底盘在世界坐标下的旋转
	float32_t beganangle_cwtw;//底盘映射在世界坐标下的初始偏角
    float32_t angle_cmtw;//底盘映射在世界坐标下的偏角
	float32_t cos;
	float32_t sin;

}Chassis_CoordinateSystem_Typedef;

extern M3508_HandleTypeDef m3508_id1;//left front motor
extern M3508_HandleTypeDef m3508_id2;//right front motor
extern M3508_HandleTypeDef m3508_id3;//right rear motor
extern M3508_HandleTypeDef m3508_id4;//left rear motor

/**
  * @func			void Chassis_Init(void)
  * @brief          底盘驱动初始化
  * @param[in]      none
  * @retval         none
  */
void Chassis_Init(void);


/**
  * @func			void Chassis_WheatWheel_Solution(
  *						int16_t chassisXvector,
  *						int16_t chassisYvectory,
  *						int16_t chassisRotatevector,
  *						int rotateK)
  * @brief          麦轮底盘运动学解算
  * @brief          底盘坐标系(俯视图)：
	/\底盘正方向			/\x(roll)
	||					||
	||					||
	||					||
		y(pitch)《========
		 
  * @param[in]      chassisXvector:底盘坐标系下，x方向下的速度
  * @param[in]      chassisYvectory:底盘坐标系下，y方向下的速度
  * @param[in]      chassisRotatevector:底盘(世界)坐标系下，底盘旋转的速度
  * @param[in]      rotateK:旋转系数(与底盘长款轴距有关k=a+b)
  * @retval         none
  */
void Chassis_WheatWheel_Solution(int16_t chassisXvector,int16_t chassisYvectory,int16_t chassisRotatevector,int rotateK);


/**
  * @func			void ChassisGyro_Task(
  *						int16_t worldXvector,
  *						int16_t worldYvector,
  *						int rotateK,
  *						float c_angle)
  * @brief          底盘小陀螺任务
  * @param[in]      worldXvector：世界/云台坐标系下，x方向速度
  * @param[in]      worldYvector：世界/云台坐标系下，y方向速度
  * @param[in]      rotateK：旋转系数(与底盘长款轴距有关k=a+b)
  * @param[in]      c_angle：世界/云台坐标系与底盘坐标系的夹角
  * @retval         none
  */
void ChassisGyro_Task(int16_t worldXvector,int16_t worldYvector,int rotateK,float c_angle);


/**
  * @func			void ChassisPositionControl_Task(
  *						int16_t worldXvector,
  *						int16_t worldYvector,
  *						int rotateK,
  *						float32_t realangle,
  *						float32_t targetangle)
  * @brief          底盘跟随云台任务
  * @param[in]      worldXvector：世界/云台坐标系下，x方向速度
  * @param[in]      worldYvector：世界/云台坐标系下，y方向速度
  * @param[in]      rotateK：旋转系数(与底盘长款轴距有关k=a+b)
  * @param[in]      realangle：  世界/云台坐标系与底盘坐标系的实时夹角(yaw轴电机绝对编码器值)
  * @param[in]      targetangle：世界/云台坐标系与底盘坐标系的目标夹角(yaw轴电机绝对编码器值)
  * @retval         none
  */
void ChassisPositionControl_Task(int16_t worldXvector,int16_t worldYvector,int rotateK,float32_t realangle,float32_t targetangle);


/**
  * @func			void Printf_ChassisMessage(int16_t targetspeed)
  * @brief          底盘调试任务
  * @param[in]      targetangle：各底盘电机目标速度
  * @retval         none
  */
void Printf_ChassisMessage(int16_t targetspeed);
#endif
