/**
  ****************************(C) COPYRIGHT 2024 征途****************************
  * @file       shoot_task.c/h
  * @brief      
  *             这里是射击任务程序，包含摩擦轮驱动，拨盘驱动等功能程序
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Jan-29-2024    chenjiangnan    
  *
  @verbatim
  ==============================================================================
	bui~bui~bui~bui~bui~bui~bui~bui~bui~bui~bui~bui~bui~bui~bui~bui~bui~bui~bui
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 征途****************************
  */
#ifndef __SHOOT_TASK_H
#define __SHOOT_TASK_H

#include "CAN_receive.h"
#include "3508_driver.h"
#include "rc_task.h"
#include "rc_receive.h"

/*摩擦轮发射速度宏*/
#define SHOT_MIN_SPEED   2000
#define SHOT_MID_SPEED   4000
#define SHOT_MAX_SPEED   7000
/*拨弹速度宏*/
#define TRIGGER_MIN_SPEED    -500
#define TRIGGER_MID_SPEED    -500
#define TRIGGER_MAX_SPEED    -500
/*保险宏*/
#define VALVE_OPEN  1
#define VALVE_CLOSE 0

enum Shoot_Mode{
	
	/*循环发射为速度控制*/
	CYCLE_FIRE_MODE,//循环发射
	/*单发为位置控制模式*/
	SINGLE_SHOT_MODE,//单发
};
typedef struct{
	
	int valve;//射击阀门
	int mode;//发射控制模式

}Shoot_ModeMessageTypedef;

/*拨盘电机句柄*/
extern M3508_HandleTypeDef trigger_motor;
/*摩擦轮电机句柄 */
extern M3508_HandleTypeDef left_frition_motor;
extern M3508_HandleTypeDef right_frition_motor;
/*射击模式相关参数*/
extern Shoot_ModeMessageTypedef shoot_modemessage;


/**
  * @func			void Shoot_Init(void)
  * @brief          射击驱动初始化
  * @param[in]      none
  * @retval         none
  */
void Shoot_Init(void);


/**
  * @func			void Shoot_Control(int16_t* downtriggeroutput,int16_t* uptriggeroutput)
  * @brief          拨盘模式切换
  * @param[in]      downtriggeroutput：下拨弹pid输出值
  * @param[in]      uptriggeroutput：上拨弹pid输出值
  * @retval         none
  */
int16_t SetFrition_Shoot_Task(M3508_HandleTypeDef* motor,int16_t targetspeed);

int16_t SetTrigger_Shoot_Task(M3508_HandleTypeDef* motor,int16_t targetspeed);

#endif
