/**
  ****************************(C) COPYRIGHT 2024 ��;****************************
  * @file       shoot_task.c/h
  * @brief      
  *             ���������������򣬰���Ħ�������������������ȹ��ܳ���
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
  ****************************(C) COPYRIGHT 2024 ��;****************************
  */
#include "shoot_task.h"

/*���̵�����*/
M2006_HandleTypeDef up_trigger_motor;
M2006_HandleTypeDef down_trigger_motor;
/*Ħ���ֵ�����*/
Snail2306_HandleTypeDef snail_LFUP;
Snail2306_HandleTypeDef snail_LFDOWN;
Snail2306_HandleTypeDef snail_RTUP;
Snail2306_HandleTypeDef snail_RTDOWN;
/*���ģʽ��ز���*/
Shoot_ModeMessageTypedef shoot_modemessage;

/**
  * @func			void Shoot_Init(void)
  * @brief          ���������ʼ��
  * @param[in]      none
  * @retval         none
  */
void Shoot_Init(void)
{
	/*��ʼ����ض�ʱ��*/
	Snail2305_Init();
	/*Ħ���ֵ����ʼ��*/
	snail_LFUP.min_setup_pwm=SNAIL_LEFT_UP_MINPWM;
	snail_LFUP.max_pwm=SNAIL_LEFT_UP_MAXPWM;
	
	snail_LFDOWN.min_setup_pwm=SNAIL_LEFT_DOWN_MINPWM;
	snail_LFDOWN.max_pwm=SNAIL_LEFT_DOWN_MAXPWM;
	
	snail_RTUP.min_setup_pwm=SNAIL_RIGHT_UP_MINPWM;
	snail_RTUP.max_pwm=SNAIL_RIGHT_UP_MAXPWM;
	
	snail_RTDOWN.min_setup_pwm=SNAIL_RIGHT_DOWN_MINPWM;
	snail_RTDOWN.max_pwm=SNAIL_RIGHT_DOWN_MAXPWM;
	/*���������ʼ��*/
	 M2006_Init(7,&down_trigger_motor,PID_POSITION);
	 M2006_Init(8,&up_trigger_motor,PID_POSITION);
	/*������Ϣ��ʼ��*/
	shoot_modemessage.valve=VALVE_CLOSE;
	shoot_modemessage.mode=-1;

}


/**
  * @func			void Shoot_Control(int16_t* downtriggeroutput,int16_t* uptriggeroutput)
  * @brief          ����ģʽ�л�
  * @param[in]      downtriggeroutput���²���pid���ֵ
  * @param[in]      uptriggeroutput���ϲ���pid���ֵ
  * @retval         none
  */
void Shoot_Control(int16_t* downtriggeroutput,int16_t* uptriggeroutput)
{
	/*��ȡ�����������*/
	M2006_GetBasicData(&up_trigger_motor);
	M2006_GetBasicData(&down_trigger_motor);
    //M2006_GetExpandECDData(&up_trigger_motor);
	if(rc_control_mode==CHASSIS_NORMAL_MODE)
	{
		switch(RC_Ctl.rc.s2)
		{
			case 1://�����ǹ�ܷ��䣬���·������������
				shoot_modemessage.valve=VALVE_OPEN;
			
				SetLeft_Gun(LEFT_SHOT_SPEED);
			    SetRight_Gun(SNAIL_ALL_STOP_PWM);
			
				*downtriggeroutput=M2006_SpeedLoopPIDController(&down_trigger_motor.speed_control_data,down_trigger_motor.basic_data.speed_rpm, DOMN_TRIGGER_SPEED );
			    *uptriggeroutput=M2006_SpeedLoopPIDController(&up_trigger_motor.speed_control_data,up_trigger_motor.basic_data.speed_rpm, 0);
				break;
			case 3://���Ҿ�������
				shoot_modemessage.valve=VALVE_CLOSE;
			
				SetLeft_Gun(SNAIL_ALL_STOP_PWM);
			    SetRight_Gun(SNAIL_ALL_STOP_PWM);
			
				*downtriggeroutput=M2006_SpeedLoopPIDController(&down_trigger_motor.speed_control_data,down_trigger_motor.basic_data.speed_rpm, 0 );
				*uptriggeroutput=M2006_SpeedLoopPIDController(&up_trigger_motor.speed_control_data,up_trigger_motor.basic_data.speed_rpm, 0);
				break;
			case 2://���Ҳ�ǹ�ܷ��䣬���Ϸ������������
				shoot_modemessage.valve=VALVE_OPEN;
			
				SetRight_Gun(RIGHT_SHOT_SPEED);
				SetLeft_Gun(SNAIL_ALL_STOP_PWM);
			
				*uptriggeroutput=M2006_SpeedLoopPIDController(&up_trigger_motor.speed_control_data,up_trigger_motor.basic_data.speed_rpm, UP_TRIGGER_SPEED);
			    *downtriggeroutput=M2006_SpeedLoopPIDController(&down_trigger_motor.speed_control_data,down_trigger_motor.basic_data.speed_rpm, 0 );
				break;
			
		}
	}
	else if(rc_control_mode==CHASSIS_GYRO_MODE)//�����Ϊ���
	{
		switch(RC_Ctl.rc.s2)
		{
			case 1://˫ǹ����
				shoot_modemessage.valve=VALVE_OPEN;
				if((RC_Ctl.rc.roller-1024)>0)//���¿۶����
				{
					/*����Ħ����*/
					SetLeft_Gun(LEFT_SHOT_SPEED);
					SetRight_Gun(RIGHT_SHOT_SPEED);
					/*��������*/
					*downtriggeroutput=M2006_SpeedLoopPIDController(&down_trigger_motor.speed_control_data,down_trigger_motor.basic_data.speed_rpm, DOMN_TRIGGER_SPEED );
					*uptriggeroutput=M2006_SpeedLoopPIDController(&up_trigger_motor.speed_control_data,up_trigger_motor.basic_data.speed_rpm, UP_TRIGGER_SPEED);
				}
				else
				{
					/*�ر�Ħ����*/
					SetLeft_Gun(SNAIL_ALL_STOP_PWM);
					SetRight_Gun(SNAIL_ALL_STOP_PWM);
					/*�رղ���*/
					*downtriggeroutput=M2006_SpeedLoopPIDController(&down_trigger_motor.speed_control_data,down_trigger_motor.basic_data.speed_rpm, 0 );
					*uptriggeroutput=M2006_SpeedLoopPIDController(&up_trigger_motor.speed_control_data,up_trigger_motor.basic_data.speed_rpm, 0);
				}
				break;
			case 3://�������ո�
				shoot_modemessage.valve=VALVE_CLOSE;
				SetLeft_Gun(SNAIL_ALL_STOP_PWM);
			    SetRight_Gun(SNAIL_ALL_STOP_PWM);
			
				*downtriggeroutput=M2006_SpeedLoopPIDController(&down_trigger_motor.speed_control_data,down_trigger_motor.basic_data.speed_rpm, 0 );
				*uptriggeroutput=M2006_SpeedLoopPIDController(&up_trigger_motor.speed_control_data,up_trigger_motor.basic_data.speed_rpm, 0);
				break;
			case 2://���෢��
				shoot_modemessage.valve=VALVE_OPEN;
				if((RC_Ctl.rc.roller-1024)>0)//���¿۶�����������ǹ�ܷ��䣬���·������������
				{
					SetLeft_Gun(LEFT_SHOT_SPEED);
					SetRight_Gun(SNAIL_ALL_STOP_PWM);
			
					*downtriggeroutput=M2006_SpeedLoopPIDController(&down_trigger_motor.speed_control_data,down_trigger_motor.basic_data.speed_rpm, DOMN_TRIGGER_SPEED );
					*uptriggeroutput=M2006_SpeedLoopPIDController(&up_trigger_motor.speed_control_data,up_trigger_motor.basic_data.speed_rpm, 0);
				}
				else if((RC_Ctl.rc.roller-1024)==0)//׼�����
				{
					/*�ر�Ħ����*/
					SetLeft_Gun(SNAIL_ALL_STOP_PWM);
					SetRight_Gun(SNAIL_ALL_STOP_PWM);
					/*�رղ������*/
					*downtriggeroutput=M2006_SpeedLoopPIDController(&down_trigger_motor.speed_control_data,down_trigger_motor.basic_data.speed_rpm, 0 );
					*uptriggeroutput=M2006_SpeedLoopPIDController(&up_trigger_motor.speed_control_data,up_trigger_motor.basic_data.speed_rpm, 0);
				}
				else//���Ͽ۶���������Ҳ�ǹ�ܷ��䣬���Ϸ������������
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
			case 1://�����ǹ�ܷ��䣬���·������������
				shoot_modemessage.valve=VALVE_OPEN;
				SetLeft_Gun(LEFT_SHOT_SPEED);
			    SetRight_Gun(SNAIL_ALL_STOP_PWM);
			
				*downtriggeroutput=M2006_SpeedLoopPIDController(&down_trigger_motor.speed_control_data,down_trigger_motor.basic_data.speed_rpm, DOMN_TRIGGER_SPEED );
			    *uptriggeroutput=M2006_SpeedLoopPIDController(&up_trigger_motor.speed_control_data,up_trigger_motor.basic_data.speed_rpm, 0);
				break;
			case 3://���Ҿ�������
				shoot_modemessage.valve=VALVE_CLOSE;
				SetLeft_Gun(SNAIL_ALL_STOP_PWM);
			    SetRight_Gun(SNAIL_ALL_STOP_PWM);
			
				*downtriggeroutput=M2006_SpeedLoopPIDController(&down_trigger_motor.speed_control_data,down_trigger_motor.basic_data.speed_rpm, 0 );
 				*uptriggeroutput=M2006_SpeedLoopPIDController(&up_trigger_motor.speed_control_data,up_trigger_motor.basic_data.speed_rpm, 0);
				break;
			case 2://���Ҳ�ǹ�ܷ��䣬���Ϸ������������
				shoot_modemessage.valve=VALVE_OPEN;
				SetRight_Gun(RIGHT_SHOT_SPEED);
				SetLeft_Gun(SNAIL_ALL_STOP_PWM);
			
				*uptriggeroutput=M2006_SpeedLoopPIDController(&up_trigger_motor.speed_control_data,up_trigger_motor.basic_data.speed_rpm, UP_TRIGGER_SPEED);
			    *downtriggeroutput=M2006_SpeedLoopPIDController(&down_trigger_motor.speed_control_data,down_trigger_motor.basic_data.speed_rpm, 0 );
				break;
		}
	}
}









