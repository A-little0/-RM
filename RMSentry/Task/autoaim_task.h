#ifndef __AUTOAIM_TASK_H
#define __AUTOAIM_TASK_H

#include "main.h"
#include "chassis_task.h"
#include "gimbal_task.h"
#include "shoot_task.h"
#include "ros_task.h"

#define AUTO_AIM_YAW_RATE 0.06//0.0015
#define AUTO_AIM_PITCH_RATE 0.7
typedef enum{
  AUTO_AIM_LOSE=0,
  AUTO_AIM_GET=1,
  AUTO_AIM_SHOOT_TURN_OFF=2,
  AUTO_AIM_SHOOT_TURN_ON=3,
	
}AutoAim_Signal;

typedef struct{
	//云台偏移量数据
	float d_yaw;
	float d_pitch;
	//自瞄识别信号
	uint8_t aim_signal;
	//发射信号
	uint8_t shoot_signal;
}AutoAim_DataHandleTypedef;

extern AutoAim_DataHandleTypedef  auto_aim;
/*自瞄相关参数*/
extern int auto_key;
extern int auto_key2;
extern int auto_tim;
extern float auto_yaw_targetpostion;
extern float auto_pitch_targetpostion;

void AutoAim_TaskInit(void);
AutoAim_Signal GetAutoAIM(void);
void AutoAim_Task(void);
#endif
