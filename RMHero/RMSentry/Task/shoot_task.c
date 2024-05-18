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
M3508_HandleTypeDef trigger_motor;
/*Ħ���ֵ����� */
M3508_HandleTypeDef left_frition_motor;
M3508_HandleTypeDef right_frition_motor;
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
	/*Ħ���ֵ����ʼ��*/
    M3508_Init(7,&left_frition_motor,PID_DELTA);
    M3508_Init(8,&right_frition_motor,PID_DELTA);
	/*���������ʼ��*/
    M3508_Init(5,&trigger_motor,PID_DELTA);
	/*������Ϣ��ʼ��*/
	shoot_modemessage.valve=VALVE_CLOSE;
	shoot_modemessage.mode=-1;

}

//Ħ����
int16_t SetFrition_Shoot_Task(M3508_HandleTypeDef* motor,int16_t targetspeed)
{
	int16_t intput=0;
	int16_t output=0;
	//��ȡ�����Ϣ
	M3508_GetBasicData(motor);
    //����Ŀ���ٶ�
	switch(0x200+motor->motor_id)
	{
		case CAN_FRITION_LEFT_MOTOR_ID:
			intput=targetspeed;
			break;
		case CAN_FRITION_RIGHT_MOTOR_ID:
			intput=-targetspeed;
			break;
		default:
			intput=0;
			break;
	}
	output=M3508_SpeedLoopPIDController(&(motor->speed_control_data),motor->basic_data.speed_rpm,intput);
	
	return output;
}
//����
int16_t SetTrigger_Shoot_Task(M3508_HandleTypeDef* motor,int16_t targetspeed)
{
	int16_t output=M3508_SpeedLoopPIDController(&(motor->speed_control_data),motor->basic_data.speed_rpm,targetspeed);
	return output;
}











