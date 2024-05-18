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
#ifndef __SHOOT_TASK_H
#define __SHOOT_TASK_H

#include "CAN_receive.h"
#include "3508_driver.h"
#include "rc_task.h"
#include "rc_receive.h"

/*Ħ���ַ����ٶȺ�*/
#define SHOT_MIN_SPEED   2000
#define SHOT_MID_SPEED   4000
#define SHOT_MAX_SPEED   7000
/*�����ٶȺ�*/
#define TRIGGER_MIN_SPEED    -500
#define TRIGGER_MID_SPEED    -500
#define TRIGGER_MAX_SPEED    -500
/*���պ�*/
#define VALVE_OPEN  1
#define VALVE_CLOSE 0

enum Shoot_Mode{
	
	/*ѭ������Ϊ�ٶȿ���*/
	CYCLE_FIRE_MODE,//ѭ������
	/*����Ϊλ�ÿ���ģʽ*/
	SINGLE_SHOT_MODE,//����
};
typedef struct{
	
	int valve;//�������
	int mode;//�������ģʽ

}Shoot_ModeMessageTypedef;

/*���̵�����*/
extern M3508_HandleTypeDef trigger_motor;
/*Ħ���ֵ����� */
extern M3508_HandleTypeDef left_frition_motor;
extern M3508_HandleTypeDef right_frition_motor;
/*���ģʽ��ز���*/
extern Shoot_ModeMessageTypedef shoot_modemessage;


/**
  * @func			void Shoot_Init(void)
  * @brief          ���������ʼ��
  * @param[in]      none
  * @retval         none
  */
void Shoot_Init(void);


/**
  * @func			void Shoot_Control(int16_t* downtriggeroutput,int16_t* uptriggeroutput)
  * @brief          ����ģʽ�л�
  * @param[in]      downtriggeroutput���²���pid���ֵ
  * @param[in]      uptriggeroutput���ϲ���pid���ֵ
  * @retval         none
  */
int16_t SetFrition_Shoot_Task(M3508_HandleTypeDef* motor,int16_t targetspeed);

int16_t SetTrigger_Shoot_Task(M3508_HandleTypeDef* motor,int16_t targetspeed);

#endif
