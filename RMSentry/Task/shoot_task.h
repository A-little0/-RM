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

#include "2006_driver.h"
#include "snail2305_driver.h"
#include "rc_task.h"
#include "rc_receive.h"

/*摩擦轮发射速度宏*/
#define LEFT_SHOT_SPEED   1300
#define RIGHT_SHOT_SPEED  1700
#define LEFT_STOP_SPEED   1400
#define RIGHT_STOP_SPEED  1400
/*拨弹速度宏*/
#define UP_TRIGGER_SPEED    -500
#define DOMN_TRIGGER_SPEED  -500
/*保险宏*/
#define VALVE_OPEN  1
#define VALVE_CLOSE 0

enum Shoot_Mode{
	
	CYCLE_FIRE_MODE,//循环发射
	SINGLE_SHOT_MODE,//单发
};
typedef struct{
	
	int valve;//射击阀门
	int mode;//发射控制模式

}Shoot_ModeMessageTypedef;

/*拨盘电机句柄*/
extern M2006_HandleTypeDef up_trigger_motor;
extern M2006_HandleTypeDef down_trigger_motor;
/*摩擦轮电机句柄*/
extern Snail2306_HandleTypeDef snail_LFUP;
extern Snail2306_HandleTypeDef snail_LFDOWN;
extern Snail2306_HandleTypeDef snail_RTUP;
extern Snail2306_HandleTypeDef snail_RTDOWN;


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
void Shoot_Control(int16_t* downtriggeroutput,int16_t* uptriggeroutput);

#endif
