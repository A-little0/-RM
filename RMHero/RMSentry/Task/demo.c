#include "demo.h"


void RobotStatus_RemoteControl()
{
	int16_t yaw_output=0;
	int16_t pitch_output=0;
	int16_t trigger1_output=0;
	int16_t trigger2_output=0;
	
	int16_t trigger3_output=0;
	int16_t g_compensation=0;
	static int mode=0;
	mode=RC_Ctl.rc.s1;

	switch(mode)
	{
		case CHASSIS_NORMAL_MODE:
			Chassis_WheatWheel_Solution(rc_user.x,rc_user.y,rc_user.z,1);
		    yaw_output=Yaw_Gimbal_Task(rc_user.yaw);
			pitch_output=Pitch_Gimbal_Task(rc_user.pitch);
		    g_compensation=GravityCompensation(&pitch_gimbal_motor);
			
		    if(RC_Ctl.rc.s2 == 2)//停止发射
			{
				trigger1_output=SetFrition_Shoot_Task(&left_frition_motor,0);
				trigger2_output=SetFrition_Shoot_Task(&right_frition_motor,0);
				M3508_GetBasicData(&trigger_motor);
				trigger3_output=SetTrigger_Shoot_Task(&trigger_motor,0);
				CAN_cmd_shoot(trigger3_output,0,0,0);
				
			}
			else if(RC_Ctl.rc.s2 == 3)//开启摩擦轮
			{
				trigger1_output=SetFrition_Shoot_Task(&left_frition_motor,SHOT_MID_SPEED);
				trigger2_output=SetFrition_Shoot_Task(&right_frition_motor,SHOT_MID_SPEED);
				M3508_GetBasicData(&trigger_motor);
				trigger3_output=SetTrigger_Shoot_Task(&trigger_motor,0);
				CAN_cmd_shoot(trigger3_output,0,0,0);

			}
			else//开启摩擦轮 并 开启拨盘
			{
				trigger1_output=SetFrition_Shoot_Task(&left_frition_motor,SHOT_MID_SPEED);
				trigger2_output=SetFrition_Shoot_Task(&right_frition_motor,SHOT_MID_SPEED);
			    M3508_GetBasicData(&trigger_motor);
				trigger3_output=SetTrigger_Shoot_Task(&trigger_motor,TRIGGER_MIN_SPEED);
				CAN_cmd_shoot(trigger3_output,0,0,0);
			}
			CAN_cmd_gimbal(pitch_output, yaw_output, trigger1_output, trigger2_output);
			break;
		case CHASSIS_FOLLOW_GIMBAL_MODE:
			ChassisPositionControl_Task(rc_user.x,rc_user.y,1,YAW_MEDIAN,yaw_gimbal_motor.basic_data.ecd);
			M6020_GetBasicData(&yaw_gimbal_motor);//获取数据
			if(yaw_key==0){
				Get_ReferenceMedianAngle(&JY901_data,&yaw_gimbal_motor);
				yaw_key=1;
				rc_user.yaw=0;
			} 
			
			yaw_output=Yaw_Gimbal_Task2(JY901_data.angle.angle[2],(yaw_beganangle+rc_user.yaw)); 
			
			pitch_output=Pitch_Gimbal_Task(rc_user.pitch);
			//g_compensation=GravityCompensation(&pitch_gimbal_motor);
			//Shoot_Control(&trigger1_output,&trigger2_output);
			CAN_cmd_gimbal(pitch_output,yaw_output, trigger1_output, trigger2_output);
			
			
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
			
			yaw_output=Yaw_Gimbal_Task2(JY901_data.angle.angle[2],(yaw_beganangle+rc_user.yaw));
			pitch_output=Pitch_Gimbal_Task(rc_user.pitch);
			//g_compensation=GravityCompensation(&pitch_gimbal_motor);
			//Shoot_Control(&trigger1_output,&trigger2_output);
			CAN_cmd_gimbal(pitch_output, yaw_output, trigger1_output, trigger2_output);
			
			ChassisGyro_Task(rc_user.x,rc_user.y,0,(YAW_MEDIAN-yaw_gimbal_motor.basic_data.ecd));
			break;
		default:
			CAN_cmd_chassis(0,0,0,0);
			g_compensation=GravityCompensation(&pitch_gimbal_motor);
			CAN_cmd_gimbal(0,g_compensation, 0, 0);
			break;
			
	}
	
	if(mode!=CHASSIS_FOLLOW_GIMBAL_MODE){yaw_key=0;}
	if(mode!=CHASSIS_GYRO_MODE){yaw_key2=0;}
	if(mode==RC_SIGNAL_UNLINK){mode=0;}
}

void RobotStatus_KeyBoardControl()
{
	int16_t yaw_output=0;
	int16_t pitch_output=0;
	int16_t trigger1_output=0;
	int16_t trigger2_output=0;
	int16_t g_compensation=0;
	static int mode=0;
	mode=keyboard_user.s1;

	switch(mode)
	{
		case CHASSIS_FOLLOW_GIMBAL_MODE:
			ChassisPositionControl_Task(keyboard_user.x,keyboard_user.y,1,yaw_gimbal_motor.basic_data.ecd,YAW_MEDIAN);
			M6020_GetBasicData(&yaw_gimbal_motor);//获取数据
			if(yaw_key==0){
				Get_ReferenceMedianAngle(&JY901_data,&yaw_gimbal_motor);
				yaw_key=1;
				rc_user.yaw=0;
			} 
			
			yaw_output=Yaw_Gimbal_Task2(JY901_data.angle.angle[2],(yaw_beganangle+keyboard_user.yaw)); 
			
			pitch_output=Pitch_Gimbal_Task(keyboard_user.pitch);
			g_compensation=GravityCompensation(&pitch_gimbal_motor);
			//Shoot_Control(&trigger1_output,&trigger2_output);
			CAN_cmd_gimbal(yaw_output,pitch_output+g_compensation, trigger1_output, trigger2_output);
			
			
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
			
			yaw_output=Yaw_Gimbal_Task2(JY901_data.angle.angle[2],(yaw_beganangle+keyboard_user.yaw));
			pitch_output=Pitch_Gimbal_Task(keyboard_user.pitch);
			g_compensation=GravityCompensation(&pitch_gimbal_motor);
			//Shoot_Control(&trigger1_output,&trigger2_output);
			CAN_cmd_gimbal(pitch_output, yaw_output+g_compensation, trigger1_output, trigger2_output);
			
			ChassisGyro_Task(keyboard_user.x,keyboard_user.y,1,YAW_MEDIAN-yaw_gimbal_motor.basic_data.ecd);//yaw_beganecd+rc_user.yaw*8191/360-yaw_gimbal_motor.basic_data.ecd//yaw_beganecd+rc_user.yaw*8191/360-yaw_gimbal_motor.basic_data.ecd)
			break;
		default:
			CAN_cmd_chassis(0,0,0,0);
			g_compensation=GravityCompensation(&pitch_gimbal_motor);
			CAN_cmd_gimbal(0,g_compensation, 0, 0);
			break;
			
	}
	
	if(mode!=CHASSIS_FOLLOW_GIMBAL_MODE){yaw_key=0;}
	if(mode!=CHASSIS_GYRO_MODE){yaw_key2=0;}
	if(mode==RC_SIGNAL_UNLINK){mode=0;}
}

uint8_t RobotStatus_FullAuto()
{
	
}

/**
  * @func			void Demo(void)
  * @brief          demo程序
  * @param[in]      none
  * @retval         none
  */
void Demo(void)
{
	switch(RC_Ctl.control_mode)
	{
		case RC_REMOTE_CONTROL_MODE:
			RobotStatus_RemoteControl();
			break;
		case KEY_BOARD_CONTROL_MODE:
			RobotStatus_KeyBoardControl();
			break;
	}
}
