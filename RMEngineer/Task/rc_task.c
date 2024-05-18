/**
  ****************************(C) COPYRIGHT 2024 ��;****************************
  * @file       rc_task.c/h
  * @brief      
  *             ������demo�������,��~
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Jan-29-2024    chenjiangnan      �д����
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 ��;****************************
  */
#include "rc_task.h"

/*ң����ģʽ���*/
RC_ControlMode rc_control_mode;


/**
  * @func			void RC_TaskInit(void)
  * @brief          ң���������ʼ��
  * @param[in]      none
  * @retval         none
  */
void RC_TaskInit(void)
{
	/*��ʼң��������ģʽ����Ϊδ����*/
	rc_control_mode=RC_SIGNAL_UNLINK;
}
/**
  * @func			void Demo(void)
  * @brief          demo����
  * @param[in]      none
  * @retval         none
  */
void Demo(void)
{
	static uint8_t chassis_mode=0;
	static uint8_t solenoid_valve_mode=0;
	chassis_mode=rc_control_mode;
	solenoid_valve_mode=RC_Ctl.rc.s2;
	/*���ƿ���*/
	switch(chassis_mode)
	{
		
		case CHASSIS_NORMAL_MODE:
			/*�Ե�������ϵΪ�ο�������ӳ��*/
			Chassis_WheatWheel_Solution(rc_user.x,rc_user.y,rc_user.z,1);
			Endeffector_Task(rc_user.updown,rc_user.pushpull,0);
			break;
		case CHASSIS_CONTROL_DISABLE_MODE:
			/*��������*/
			Chassis_WheatWheel_Solution(0,0,0,1);
			Endeffector_Task(rc_user.x,rc_user.y,rc_user.z);
			break;
		case CHASSIS_GYRO_MODE:
			/*����������ϵΪ�ο�������ӳ��*/
//			ChassisPositionControl_Task();
//		    ChassisCoordinateMap_Task();
			//Endeffector_Task(rc_user.updown,rc_user.pushpull,0);
			break;
		default:
			break;

	}
	/*���̿���*/
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



