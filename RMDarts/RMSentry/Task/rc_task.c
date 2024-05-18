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

float target_angle_test=0.0f;
int16_t target_position=YAW_MEDIAN;
int16_t  chassis_angle_test=0;

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

void RC_Remote_Process(int mode)
{
    switch(mode)
	{
		case 1:
			rc_control_mode=CHASSIS_NORMAL_MODE;
			/*�ٶ�ʹ�þ���ֵ*/
			rc_user.x     =  ( RC_Ctl.rc.ch3-1024 ) * RC_sent_X;//ͨ��2��������
			rc_user.y     =  ( RC_Ctl.rc.ch2-1024 ) * RC_sent_Y;//ͨ��3����ǰ��
			rc_user.z     = -( RC_Ctl.rc.roller -1024) * RC_sent_rotate;//ͨ��roller������Բת��
			/*�Ƕ�ʹ������*/
			rc_user.yaw  =   -( RC_Ctl.rc.ch0-1024 ) +YAW_MEDIAN;//ͨ��0������̨yaw��ת
			rc_user.pitch += ( RC_Ctl.rc.ch1-1024 ) * RC_sent_pitch;//ͨ��1������̨pitch��ת
			break;
		case 3:
			rc_control_mode=CHASSIS_FOLLOW_GIMBAL_MODE;
			/*�ٶ�ʹ�þ���ֵ*/
			rc_user.x     =  ( RC_Ctl.rc.ch3-1024 ) * RC_sent_X;//ͨ��2��������
			rc_user.y     =  ( RC_Ctl.rc.ch2-1024 ) * RC_sent_Y;//ͨ��3����ǰ��
			/*�Ƕ�ʹ������*/
			rc_user.yaw   += ( RC_Ctl.rc.roller-1024 )*RC_sent_yaw/2;//ͨ��roller������̨yaw��ת
			rc_user.pitch += ( RC_Ctl.rc.ch1-1024 ) * RC_sent_pitch;//ͨ��1������̨pitch��ת
			break;
		case 2:
			rc_control_mode=CHASSIS_GYRO_MODE;
			/*�ٶ�ʹ�þ���ֵ*/
			rc_user.x     =  ( RC_Ctl.rc.ch3-1024 ) * RC_sent_X;//ͨ��2��������
			rc_user.y     =  ( RC_Ctl.rc.ch2-1024 ) * RC_sent_Y;//ͨ��3����ǰ��
			rc_user.z     =  ( RC_Ctl.rc.roller -1024) * RC_sent_rotate;//ͨ��roller������Բת��
			/*�Ƕ�ʹ������*/
			rc_user.yaw   -= ( RC_Ctl.rc.ch0-1024 )*RC_sent_yaw;//ͨ��0������̨yaw��ת
			rc_user.pitch += ( RC_Ctl.rc.ch1-1024 )*RC_sent_pitch;//ͨ��1������̨pitch��ת
			break;
		default://δ���յ�ң������ֵ
			rc_control_mode=RC_SIGNAL_UNLINK;
			RC_Receive_Init();
			break;
	}
	/*��yaw���޷�*/
	/*��pitch���޷�*/
	if(rc_user.pitch>PITCH_MAXANGLE){rc_user.pitch=PITCH_MAXANGLE;}
	else if(rc_user.pitch<PITCH_MINANGLE){rc_user.pitch=PITCH_MINANGLE;}
}

void KeyBoard_Process(int mode)
{
    switch(mode)
	{
		case CHASSIS_FOLLOW_GIMBAL_MODE:
		case CHASSIS_GYRO_MODE:{
			/*�ٶ�ʹ������*/
			if((RC_Ctl.key.v & KEY_PRESSED_OFFSET_W ) + (RC_Ctl.key.v & KEY_PRESSED_OFFSET_S ) == 0){
				keyboard_user.x=0;
			}
			else if((RC_Ctl.key.v & KEY_PRESSED_OFFSET_W )!= 0){
				keyboard_user.x+=KEY_SENT_CHASSIS_X;
				if(keyboard_user.x>CHASSIS_MAX_XVECTOR_SPEED){keyboard_user.x=CHASSIS_MAX_XVECTOR_SPEED;}
				
			}
			else if((RC_Ctl.key.v & KEY_PRESSED_OFFSET_S )!= 0){
				keyboard_user.x-=KEY_SENT_CHASSIS_X;
				if(-keyboard_user.x < -CHASSIS_MAX_XVECTOR_SPEED){keyboard_user.x=-CHASSIS_MAX_XVECTOR_SPEED;}
			}
			
			if((RC_Ctl.key.v & KEY_PRESSED_OFFSET_A ) + (RC_Ctl.key.v & KEY_PRESSED_OFFSET_D ) == 0){
				keyboard_user.y=0;
			}
			else if((RC_Ctl.key.v & KEY_PRESSED_OFFSET_A )!= 0){
				keyboard_user.y-=KEY_SENT_CHASSIS_Y;
				if(keyboard_user.y > CHASSIS_MAX_YVECTOR_SPEED){keyboard_user.y=CHASSIS_MAX_YVECTOR_SPEED;}
			}
			else if((RC_Ctl.key.v & KEY_PRESSED_OFFSET_D )!= 0){
				keyboard_user.y+=KEY_SENT_CHASSIS_Y;
				if(-keyboard_user.y < -CHASSIS_MAX_YVECTOR_SPEED){keyboard_user.y=-CHASSIS_MAX_YVECTOR_SPEED;}
			}
			/*�Ƕ�ʹ������*/
			keyboard_user.yaw   -= RC_Ctl.mouse.x*KEY_SENT_GIMBAL_YAW;//ͨ��roller������̨yaw��ת
			keyboard_user.pitch -= RC_Ctl.mouse.y*KEY_SENT_GIMBAL_PITCH;//ͨ��1������̨pitch��ת
		}break;
		case AUTO_AIM_MODE:
			
			break;
	}
	/*��yaw���޷�*/
	/*��pitch���޷�*/
	if(keyboard_user.pitch>PITCH_MAXANGLE){keyboard_user.pitch=PITCH_MAXANGLE;}
	else if(keyboard_user.pitch<PITCH_MINANGLE){keyboard_user.pitch=PITCH_MINANGLE;}
}

/**
  * @func			void RC_Process(void)
  * @brief          �����������
  * @param[in]      none
  * @retval         none
  */
void RC_Process(void)
{
	/*�ж�RC����ģʽ*/
	 static uint8_t choice_mode=0;
	 if((RC_Ctl.key.v & KEY_PRESSED_OFFSET_CTRL) != 0){
		 choice_mode++;
		 if((choice_mode & 0x01) == 1)
		 {
			RC_Ctl.control_mode=KEY_BOARD_CONTROL_MODE;
		 }
		 else
		 {
			RC_Ctl.control_mode=RC_REMOTE_CONTROL_MODE;
		 }
	}
	 
	/*RC�������ѻ���*/
	if(RC_Ctl.rc.s1!=rc_user.last_s1){
		RC_Ctl.control_mode=RC_REMOTE_CONTROL_MODE;
		choice_mode=0;
	}
	
	/*����ң������������*/
	RC_Remote_Process(RC_Ctl.rc.s1);

	/*���������������*/
	if((RC_Ctl.key.v & KEY_PRESSED_OFFSET_Q )!= 0){keyboard_user.s1=CHASSIS_FOLLOW_GIMBAL_MODE;}
	else if((RC_Ctl.key.v & KEY_PRESSED_OFFSET_E )!= 0 ){keyboard_user.s1=CHASSIS_GYRO_MODE;}
	KeyBoard_Process(keyboard_user.s1);

	rc_user.last_s1=RC_Ctl.rc.s1;
}



