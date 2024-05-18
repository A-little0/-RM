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
#include "shoot_task.h"

/*拨盘电机句柄*/
M2006_HandleTypeDef up_trigger_motor;
M2006_HandleTypeDef down_trigger_motor;
/*摩擦轮电机句柄*/
Snail2306_HandleTypeDef snail_LFUP;
Snail2306_HandleTypeDef snail_LFDOWN;
Snail2306_HandleTypeDef snail_RTUP;
Snail2306_HandleTypeDef snail_RTDOWN;
/*射击模式相关参数*/
Shoot_ModeMessageTypedef shoot_modemessage;

/**
  * @func			void Shoot_Init(void)
  * @brief          射击驱动初始化
  * @param[in]      none
  * @retval         none
  */
void Shoot_Init(void)
{
	/*初始化相关定时器*/
	Snail2305_Init();
	/*摩擦轮电机初始化*/
	snail_LFUP.min_setup_pwm=SNAIL_LEFT_UP_MINPWM;
	snail_LFUP.max_pwm=SNAIL_LEFT_UP_MAXPWM;
	
	snail_LFDOWN.min_setup_pwm=SNAIL_LEFT_DOWN_MINPWM;
	snail_LFDOWN.max_pwm=SNAIL_LEFT_DOWN_MAXPWM;
	
	snail_RTUP.min_setup_pwm=SNAIL_RIGHT_UP_MINPWM;
	snail_RTUP.max_pwm=SNAIL_RIGHT_UP_MAXPWM;
	
	snail_RTDOWN.min_setup_pwm=SNAIL_RIGHT_DOWN_MINPWM;
	snail_RTDOWN.max_pwm=SNAIL_RIGHT_DOWN_MAXPWM;
	/*拨弹电机初始化*/
	 M2006_Init(7,&down_trigger_motor,PID_POSITION);
	 M2006_Init(8,&up_trigger_motor,PID_POSITION);
	/*拨弹信息初始化*/
	shoot_modemessage.valve=VALVE_CLOSE;
	shoot_modemessage.mode=-1;

}


/**
  * @func			void Shoot_Control(int16_t* downtriggeroutput,int16_t* uptriggeroutput)
  * @brief          拨盘模式切换
  * @param[in]      downtriggeroutput：下拨弹pid输出值
  * @param[in]      uptriggeroutput：上拨弹pid输出值
  * @retval         none
  */
void Shoot_Control(int16_t* downtriggeroutput,int16_t* uptriggeroutput)
{
	/*获取电机基础数据*/
	M2006_GetBasicData(&up_trigger_motor);
	M2006_GetBasicData(&down_trigger_motor);
    //M2006_GetExpandECDData(&up_trigger_motor);
	if(rc_control_mode==CHASSIS_NORMAL_MODE)
	{
		switch(RC_Ctl.rc.s2)
		{
			case 1://仅左侧枪管发射，与下方拨弹电机联动
				shoot_modemessage.valve=VALVE_OPEN;
			
				SetLeft_Gun(LEFT_SHOT_SPEED);
			    SetRight_Gun(SNAIL_ALL_STOP_PWM);
			
				*downtriggeroutput=M2006_SpeedLoopPIDController(&down_trigger_motor.speed_control_data,down_trigger_motor.basic_data.speed_rpm, DOMN_TRIGGER_SPEED );
			    *uptriggeroutput=M2006_SpeedLoopPIDController(&up_trigger_motor.speed_control_data,up_trigger_motor.basic_data.speed_rpm, 0);
				break;
			case 3://左右均不发射
				shoot_modemessage.valve=VALVE_CLOSE;
			
				SetLeft_Gun(SNAIL_ALL_STOP_PWM);
			    SetRight_Gun(SNAIL_ALL_STOP_PWM);
			
				*downtriggeroutput=M2006_SpeedLoopPIDController(&down_trigger_motor.speed_control_data,down_trigger_motor.basic_data.speed_rpm, 0 );
				*uptriggeroutput=M2006_SpeedLoopPIDController(&up_trigger_motor.speed_control_data,up_trigger_motor.basic_data.speed_rpm, 0);
				break;
			case 2://仅右侧枪管发射，与上方拨弹电机联动
				shoot_modemessage.valve=VALVE_OPEN;
			
				SetRight_Gun(RIGHT_SHOT_SPEED);
				SetLeft_Gun(SNAIL_ALL_STOP_PWM);
			
				*uptriggeroutput=M2006_SpeedLoopPIDController(&up_trigger_motor.speed_control_data,up_trigger_motor.basic_data.speed_rpm, UP_TRIGGER_SPEED);
			    *downtriggeroutput=M2006_SpeedLoopPIDController(&down_trigger_motor.speed_control_data,down_trigger_motor.basic_data.speed_rpm, 0 );
				break;
			
		}
	}
	else if(rc_control_mode==CHASSIS_GYRO_MODE)//发射改为扳机
	{
		switch(RC_Ctl.rc.s2)
		{
			case 1://双枪齐下
				shoot_modemessage.valve=VALVE_OPEN;
				if((RC_Ctl.rc.roller-1024)>0)//向下扣动扳机
				{
					/*开启摩擦轮*/
					SetLeft_Gun(LEFT_SHOT_SPEED);
					SetRight_Gun(RIGHT_SHOT_SPEED);
					/*开启拨盘*/
					*downtriggeroutput=M2006_SpeedLoopPIDController(&down_trigger_motor.speed_control_data,down_trigger_motor.basic_data.speed_rpm, DOMN_TRIGGER_SPEED );
					*uptriggeroutput=M2006_SpeedLoopPIDController(&up_trigger_motor.speed_control_data,up_trigger_motor.basic_data.speed_rpm, UP_TRIGGER_SPEED);
				}
				else
				{
					/*关闭摩擦轮*/
					SetLeft_Gun(SNAIL_ALL_STOP_PWM);
					SetRight_Gun(SNAIL_ALL_STOP_PWM);
					/*关闭拨盘*/
					*downtriggeroutput=M2006_SpeedLoopPIDController(&down_trigger_motor.speed_control_data,down_trigger_motor.basic_data.speed_rpm, 0 );
					*uptriggeroutput=M2006_SpeedLoopPIDController(&up_trigger_motor.speed_control_data,up_trigger_motor.basic_data.speed_rpm, 0);
				}
				break;
			case 3://开启保险杠
				shoot_modemessage.valve=VALVE_CLOSE;
				SetLeft_Gun(SNAIL_ALL_STOP_PWM);
			    SetRight_Gun(SNAIL_ALL_STOP_PWM);
			
				*downtriggeroutput=M2006_SpeedLoopPIDController(&down_trigger_motor.speed_control_data,down_trigger_motor.basic_data.speed_rpm, 0 );
				*uptriggeroutput=M2006_SpeedLoopPIDController(&up_trigger_motor.speed_control_data,up_trigger_motor.basic_data.speed_rpm, 0);
				break;
			case 2://单侧发射
				shoot_modemessage.valve=VALVE_OPEN;
				if((RC_Ctl.rc.roller-1024)>0)//向下扣动扳机，仅左侧枪管发射，与下方拨弹电机联动
				{
					SetLeft_Gun(LEFT_SHOT_SPEED);
					SetRight_Gun(SNAIL_ALL_STOP_PWM);
			
					*downtriggeroutput=M2006_SpeedLoopPIDController(&down_trigger_motor.speed_control_data,down_trigger_motor.basic_data.speed_rpm, DOMN_TRIGGER_SPEED );
					*uptriggeroutput=M2006_SpeedLoopPIDController(&up_trigger_motor.speed_control_data,up_trigger_motor.basic_data.speed_rpm, 0);
				}
				else if((RC_Ctl.rc.roller-1024)==0)//准备射击
				{
					/*关闭摩擦轮*/
					SetLeft_Gun(SNAIL_ALL_STOP_PWM);
					SetRight_Gun(SNAIL_ALL_STOP_PWM);
					/*关闭拨弹电机*/
					*downtriggeroutput=M2006_SpeedLoopPIDController(&down_trigger_motor.speed_control_data,down_trigger_motor.basic_data.speed_rpm, 0 );
					*uptriggeroutput=M2006_SpeedLoopPIDController(&up_trigger_motor.speed_control_data,up_trigger_motor.basic_data.speed_rpm, 0);
				}
				else//向上扣动扳机，仅右侧枪管发射，与上方拨弹电机联动
				{
					SetRight_Gun(RIGHT_SHOT_SPEED);
					SetLeft_Gun(SNAIL_ALL_STOP_PWM);
			
					*uptriggeroutput=M2006_SpeedLoopPIDController(&up_trigger_motor.speed_control_data,up_trigger_motor.basic_data.speed_rpm, UP_TRIGGER_SPEED);
					*downtriggeroutput=M2006_SpeedLoopPIDController(&down_trigger_motor.speed_control_data,down_trigger_motor.basic_data.speed_rpm, 0 );
				}
				break;
		}
	}
	else if(rc_control_mode==CHASSIS_FOLLOW_GIMBAL_MODE)
	{
		switch(RC_Ctl.rc.s2)
		{
			case 1://仅左侧枪管发射，与下方拨弹电机联动
				shoot_modemessage.valve=VALVE_OPEN;
				SetLeft_Gun(LEFT_SHOT_SPEED);
			    SetRight_Gun(SNAIL_ALL_STOP_PWM);
			
				*downtriggeroutput=M2006_SpeedLoopPIDController(&down_trigger_motor.speed_control_data,down_trigger_motor.basic_data.speed_rpm, DOMN_TRIGGER_SPEED );
			    *uptriggeroutput=M2006_SpeedLoopPIDController(&up_trigger_motor.speed_control_data,up_trigger_motor.basic_data.speed_rpm, 0);
				break;
			case 3://左右均不发射
				shoot_modemessage.valve=VALVE_CLOSE;
				SetLeft_Gun(SNAIL_ALL_STOP_PWM);
			    SetRight_Gun(SNAIL_ALL_STOP_PWM);
			
				*downtriggeroutput=M2006_SpeedLoopPIDController(&down_trigger_motor.speed_control_data,down_trigger_motor.basic_data.speed_rpm, 0 );
 				*uptriggeroutput=M2006_SpeedLoopPIDController(&up_trigger_motor.speed_control_data,up_trigger_motor.basic_data.speed_rpm, 0);
				break;
			case 2://仅右侧枪管发射，与上方拨弹电机联动
				shoot_modemessage.valve=VALVE_OPEN;
				SetRight_Gun(RIGHT_SHOT_SPEED);
				SetLeft_Gun(SNAIL_ALL_STOP_PWM);
			
				*uptriggeroutput=M2006_SpeedLoopPIDController(&up_trigger_motor.speed_control_data,up_trigger_motor.basic_data.speed_rpm, UP_TRIGGER_SPEED);
			    *downtriggeroutput=M2006_SpeedLoopPIDController(&down_trigger_motor.speed_control_data,down_trigger_motor.basic_data.speed_rpm, 0 );
				break;
		}
	}
}









