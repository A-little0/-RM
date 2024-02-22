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

float target_angle_test=0.0f;
int16_t target_position=YAW_MEDIAN;

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
	int16_t yaw_output=0;
	int16_t pitch_output=0;
	int16_t trigger1_output=0;
	int16_t trigger2_output=0;
	int16_t g_compensation=0;
	static int mode=0;
	mode=RC_Ctl.rc.s1;
//	if(rc_user.status == 1 )
//	{
//		rc_user.status=0;
//	}
//	else
//	{
//		mode=RC_SIGNAL_UNLINK;
//	}
	switch(mode)
	{
		case CHASSIS_NORMAL_MODE:
			Chassis_WheatWheel_Solution(rc_user.x,rc_user.y,rc_user.z,1);
		    yaw_output=Yaw_Gimbal_Task(rc_user.yaw);
			pitch_output=Pitch_Gimbal_Task(rc_user.pitch);
		    g_compensation=GravityCompensation(&pitch_gimbal_motor);
			Shoot_Control(&trigger1_output,&trigger2_output);
			CAN_cmd_gimbal(yaw_output,g_compensation+pitch_output , trigger1_output, trigger2_output);
			break;
		case CHASSIS_FOLLOW_GIMBAL_MODE:
			M6020_GetBasicData(&yaw_gimbal_motor);//获取数据
			if(yaw_key==0){
				Get_ReferenceMedianAngle(&JY901_data,&yaw_gimbal_motor);yaw_key=1;
				rc_user.yaw=0;
			} 
			target_angle_test=rc_user.yaw;
			yaw_output=Yaw_Gimbal_Task2(JY901_data.angle.angle[2],(yaw_beganangle+rc_user.yaw)); 
			pitch_output=Pitch_Gimbal_Task(rc_user.pitch);
			g_compensation=GravityCompensation(&pitch_gimbal_motor);
			Shoot_Control(&trigger1_output,&trigger2_output);
			CAN_cmd_gimbal(yaw_output,g_compensation+pitch_output, trigger1_output, trigger2_output);
			
			ChassisPositionControl_Task(rc_user.x,rc_user.y,1,yaw_gimbal_motor.basic_data.ecd,YAW_MEDIAN);
			break;
		case CHASSIS_GYRO_MODE:
			M6020_GetBasicData(&yaw_gimbal_motor);//获取数据
			if(yaw_key2==0)
			{
				Get_ReferenceMedianAngle(&JY901_data,&yaw_gimbal_motor);
				Update_ReferenceBeganeECD(&yaw_gimbal_motor);
				yaw_key2=1;
				rc_user.yaw=0;
			} 
			target_angle_test=rc_user.yaw;
			yaw_output=Yaw_Gimbal_Task2(JY901_data.angle.angle[2],(yaw_beganangle+rc_user.yaw));
			pitch_output=Pitch_Gimbal_Task(rc_user.pitch);
			g_compensation=GravityCompensation(&pitch_gimbal_motor);
			Shoot_Control(&trigger1_output,&trigger2_output);
			CAN_cmd_gimbal(yaw_output, pitch_output+g_compensation, trigger1_output, trigger2_output);
			
			ChassisGyro_Task(rc_user.x,rc_user.y,1,YAW_MEDIAN-yaw_gimbal_motor.basic_data.ecd);//yaw_beganecd+rc_user.yaw*8191/360-yaw_gimbal_motor.basic_data.ecd//yaw_beganecd+rc_user.yaw*8191/360-yaw_gimbal_motor.basic_data.ecd)
			break;
		default:
			Chassis_WheatWheel_Solution(0,0,0,0);
			g_compensation=GravityCompensation(&pitch_gimbal_motor);
			CAN_cmd_gimbal(0,g_compensation, 0, 0);
			break;
			
	}
	if(mode!=CHASSIS_FOLLOW_GIMBAL_MODE){yaw_key=0;}
	if(mode!=CHASSIS_GYRO_MODE){yaw_key2=0;}
	if(mode==RC_SIGNAL_UNLINK){mode=0;}
}



