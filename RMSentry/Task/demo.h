#ifndef __DEMO_H
#define __DEMO_H

#include "main.h"
#include "chassis_task.h"
#include "gimbal_task.h"
#include "shoot_task.h"
#include "ros_task.h"
#include "detection_task.h"
#include "autoaim_task.h"

typedef enum{
	ROBOT_REMOTE_CONTROL_STATUS,
	ROBOT_KEYBOARD_CONTROL_STATUS,
	ROBOT_FULLAUTO,
}Robot_Status;


/**
  * @func			void Demo(void)
  * @brief          demo³ÌÐò
  * @param[in]      none
  * @retval         none
  */
void Demo(void);
#endif
