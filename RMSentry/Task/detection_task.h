#ifndef  __DETECTION_H
#define  __DETECTION_H
#include "main.h"
#include "chassis_task.h"
typedef struct{
	float chassis_instantpower;
	float gimbal_instantpower;
	float shoot_instantpower;
	
	float chassis_averagepower;
	float gimbal_averagepower;
	float shoot_averagepower;
	
}RobotPowerData_HandleTypeDef;
extern int32_t rc_link_time;
extern int32_t rc_link_lasttime;

void Detection_TaskInit(void);
void Detection_RobotPowerTask(void);
#endif

