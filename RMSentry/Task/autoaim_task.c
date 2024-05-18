/**
  ****************************(C) COPYRIGHT 2024 征途****************************
  * @file      computer_driver.c/h
  * @brief      
  *             这里是自瞄接口函数，云台巡航与发射决策.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2024/4/3    chenjiangnan        1.添加视觉自瞄函数接口
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 征途****************************
 */
#include "autoaim_task.h"

AutoAim_DataHandleTypedef  auto_aim;
/*自瞄相关参数*/
int auto_key;
int auto_key2;
int auto_tim;
float auto_yaw_targetpostion;
float auto_pitch_targetpostion;

void AutoAim_TaskInit(void)
{
	/*数据初始化*/
	auto_aim.d_pitch=0;//偏移量初始化为零
	auto_aim.d_yaw=0;//偏移量初始化为零
	auto_aim.aim_signal=AUTO_AIM_LOSE;//接收信号初始化为丢失目标
	auto_aim.shoot_signal=AUTO_AIM_SHOOT_TURN_OFF;//发射信号初始化为关闭
	/*自瞄相关参数*/
	auto_key=0;
	auto_key2=0;
	auto_tim=0;
	auto_yaw_targetpostion=YAW_MEDIAN;
	auto_pitch_targetpostion=PITCH_MEDIAN;
}

 AutoAim_Signal GetAutoAIM(void)
 {
	if(auto_aim.aim_signal == AUTO_AIM_GET)//识别到装甲板
	{
		return AUTO_AIM_GET;
	}
	else//没有识别到装甲板
	{
		return AUTO_AIM_LOSE;
	}
 }
 
 void AutoAim_Task(void)
 {
	if(GetAutoAIM() == AUTO_AIM_GET)
	{
			M6020_GetBasicData(&yaw_gimbal_motor);//获取数据
			/*承接其他模式下的目标值*/
			if(auto_key==0)
			{
				Get_ReferenceMedianAngle(&JY901_data,&yaw_gimbal_motor);
				Update_ReferenceBeganECD(&yaw_gimbal_motor);
				Update_ReferenceBeganECD(&pitch_gimbal_motor);
				auto_key=1;
				if(auto_key2 == 1){auto_key2=0;}
				auto_yaw_targetpostion=yaw_beganangle;
				auto_pitch_targetpostion=pitch_beganecd;
			} 
				Get_ReferenceMedianAngle(&JY901_data,&yaw_gimbal_motor);
				Update_ReferenceBeganECD(&yaw_gimbal_motor);
				Update_ReferenceBeganECD(&pitch_gimbal_motor);	
				auto_yaw_targetpostion=yaw_beganangle;
				auto_pitch_targetpostion=pitch_beganecd;			
			
			/*目标值更新*/
			auto_yaw_targetpostion+=auto_aim.d_yaw*AUTO_AIM_YAW_RATE;//角度
			auto_pitch_targetpostion+=auto_aim.d_pitch*AUTO_AIM_PITCH_RATE;//编码器值
			/*目标值限幅*/
			if(auto_pitch_targetpostion>=PITCH_MAXANGLE){auto_pitch_targetpostion=PITCH_MAXANGLE;}//只对pitch值限幅
			if(auto_pitch_targetpostion<=PITCH_MINANGLE){auto_pitch_targetpostion=PITCH_MINANGLE;}
			int16_t yaw_output=Yaw_Gimbal_Task2(JY901_data.angle.angle[2],auto_yaw_targetpostion);
			int16_t pitch_output=Pitch_Gimbal_Task(auto_pitch_targetpostion);//auto_pitch_targetpostion
			int16_t g_compensation=GravityCompensation(&pitch_gimbal_motor);
			int16_t trigger1_output=0;
			int16_t trigger2_output=0;
			/*发射决策判断*/
			if(auto_aim.shoot_signal == AUTO_AIM_SHOOT_TURN_ON)
			{
				//启动摩擦轮
				SetRight_Gun(RIGHT_SHOT_SPEED);
				SetLeft_Gun(LEFT_SHOT_SPEED);
				//设置拨弹转速
				trigger1_output=M2006_SpeedLoopPIDController(
					&up_trigger_motor.speed_control_data,
					up_trigger_motor.basic_data.speed_rpm, 
					UP_TRIGGER_SPEED
				);
				trigger2_output=M2006_SpeedLoopPIDController(
					&down_trigger_motor.speed_control_data,
					down_trigger_motor.basic_data.speed_rpm, 
					DOMN_TRIGGER_SPEED 
				);
			}
			else
			{
				SetLeft_Gun(SNAIL_ALL_STOP_PWM);
				SetRight_Gun(SNAIL_ALL_STOP_PWM);
				trigger1_output=M2006_SpeedLoopPIDController(
					&up_trigger_motor.speed_control_data,
					up_trigger_motor.basic_data.speed_rpm, 
					0
				);
				trigger2_output=M2006_SpeedLoopPIDController(
					&down_trigger_motor.speed_control_data,
					down_trigger_motor.basic_data.speed_rpm, 
					0 
				);
			}
			
			     //yaw_output
			CAN_cmd_gimbal(0,pitch_output+g_compensation , trigger1_output, trigger2_output);//pitch_output+g_compensation
            /*小陀螺.ROS*/			
			ChassisGyro_Task(ros_data.x,ros_data.y,1,YAW_MEDIAN-yaw_gimbal_motor.basic_data.ecd);
	}
	else
	{
		    auto_tim++;
		    if(auto_tim>=20)
			{
				auto_yaw_targetpostion+=10;
				
				static int way=0;
				if(way==0)
				{
					auto_pitch_targetpostion+=50;
					if(auto_pitch_targetpostion>=PITCH_MAXANGLE)
					{
						way=1;
					}
				}
				else
				{
					auto_pitch_targetpostion-=50;
					if(auto_pitch_targetpostion<=PITCH_MINANGLE)
					{
						way=0;
					}
				}
				auto_tim=0;
			}
			//printf("%d,%d\n",auto_yaw_targetpostion,auto_pitch_targetpostion);
	
			M6020_GetBasicData(&yaw_gimbal_motor);//获取数据
			if(auto_key2==0)
			{
				Get_ReferenceMedianAngle(&JY901_data,&yaw_gimbal_motor);
				Update_ReferenceBeganECD(&yaw_gimbal_motor);
				auto_key2=1;
				if(auto_key == 1){auto_key=0;}
				//rc_user.yaw=0;
			} 
//				Get_ReferenceMedianAngle(&JY901_data,&yaw_gimbal_motor);
//				Update_ReferenceBeganECD(&yaw_gimbal_motor);
//				Update_ReferenceBeganECD(&pitch_gimbal_motor);	
//				auto_yaw_targetpostion=yaw_beganangle;
//				auto_pitch_targetpostion=pitch_beganecd;
			printf("%f\n",auto_yaw_targetpostion);
			int16_t yaw_output=Yaw_Gimbal_Task2(JY901_data.angle.angle[2],(auto_yaw_targetpostion));
			int16_t pitch_output=Pitch_Gimbal_Task(auto_pitch_targetpostion);
			int16_t g_compensation=GravityCompensation(&pitch_gimbal_motor);
			int16_t trigger1_output=M2006_SpeedLoopPIDController(
				&up_trigger_motor.speed_control_data,
				up_trigger_motor.basic_data.speed_rpm, 
				0
			);
			int16_t trigger2_output=M2006_SpeedLoopPIDController(
				&down_trigger_motor.speed_control_data,
				down_trigger_motor.basic_data.speed_rpm, 
				0 
			);
			CAN_cmd_gimbal(yaw_output, pitch_output+g_compensation, trigger1_output, trigger2_output);//yaw_output
	
			ChassisGyro_Task(0,0,1,YAW_MEDIAN-yaw_gimbal_motor.basic_data.ecd);
	}
 }
