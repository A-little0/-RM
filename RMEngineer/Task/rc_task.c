/**
  ****************************(C) COPYRIGHT 2024 征途****************************
  * @file       rc_task.c/h
  * @brief      
  *             这里是demo任务程序,嗯~
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Jan-29-2024    chenjiangnan      有待提高
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 征途****************************
  */
#include "rc_task.h"

/*遥控器模式句柄*/
RC_ControlMode rc_control_mode;


/**
  * @func			void RC_TaskInit(void)
  * @brief          遥控器任务初始化
  * @param[in]      none
  * @retval         none
  */
void RC_TaskInit(void)
{
	/*初始遥控器控制模式设置为未链接*/
	rc_control_mode=RC_SIGNAL_UNLINK;
}
/**
  * @func			void Demo(void)
  * @brief          demo程序
  * @param[in]      none
  * @retval         none
  */
void Demo(void)
{
	static uint8_t chassis_mode=0;
	static uint8_t solenoid_valve_mode=0;
	chassis_mode=rc_control_mode;
	solenoid_valve_mode=RC_Ctl.rc.s2;
	/*底云控制*/
	switch(chassis_mode)
	{
		
		case CHASSIS_NORMAL_MODE:
			/*以底盘坐标系为参考的坐标映射*/
			Chassis_WheatWheel_Solution(rc_user.x,rc_user.y,rc_user.z,1);
			Endeffector_Task(rc_user.updown,rc_user.pushpull,0);
			break;
		case CHASSIS_CONTROL_DISABLE_MODE:
			/*底盘锁死*/
			Chassis_WheatWheel_Solution(0,0,0,1);
			Endeffector_Task(rc_user.x,rc_user.y,rc_user.z);
			break;
		case CHASSIS_GYRO_MODE:
			/*以世界坐标系为参考的坐标映射*/
//			ChassisPositionControl_Task();
//		    ChassisCoordinateMap_Task();
			//Endeffector_Task(rc_user.updown,rc_user.pushpull,0);
			break;
		default:
			break;

	}
	/*吸盘控制*/
	switch(solenoid_valve_mode)
	{
		case AIR_PUMP_POWER_ENABLE_MODE:
			SolenoidValve_Control(SOLENOID_VALVE_CLOSE);
			AirPump_Control(AIR_PUMP_POWER_OPEN);
			break;
		case AIR_PUMP_POWER_DISABLE_MODE:
			SolenoidValve_Control(SOLENOID_VALVE_CLOSE);
			AirPump_Control(AIR_PUMP_POWER_CLOSE);
			break;
		case SOLENOID_VALVE_ENABLE_MODE:
			SolenoidValve_Control(SOLENOID_VALVE_OPEN);
			AirPump_Control(AIR_PUMP_POWER_CLOSE);			
			break;
		default:
		break;
	}
	
}



