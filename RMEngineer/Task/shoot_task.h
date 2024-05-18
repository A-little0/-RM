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

#include "2006_driver.h"
#include "snail2305_driver.h"
#include "rc_task.h"
#include "rc_receive.h"

/*Ħ���ַ����ٶȺ�*/
#define LEFT_SHOT_SPEED   1300
#define RIGHT_SHOT_SPEED  1700
#define LEFT_STOP_SPEED   1400
#define RIGHT_STOP_SPEED  1400
/*�����ٶȺ�*/
#define UP_TRIGGER_SPEED    -500
#define DOMN_TRIGGER_SPEED  -500
/*���պ�*/
#define VALVE_OPEN  1
#define VALVE_CLOSE 0

enum Shoot_Mode{
	
	CYCLE_FIRE_MODE,//ѭ������
	SINGLE_SHOT_MODE,//����
};
typedef struct{
	
	int valve;//�������
	int mode;//�������ģʽ

}Shoot_ModeMessageTypedef;

/*���̵�����*/
extern M2006_HandleTypeDef up_trigger_motor;
extern M2006_HandleTypeDef down_trigger_motor;
/*Ħ���ֵ�����*/
extern Snail2306_HandleTypeDef snail_LFUP;
extern Snail2306_HandleTypeDef snail_LFDOWN;
extern Snail2306_HandleTypeDef snail_RTUP;
extern Snail2306_HandleTypeDef snail_RTDOWN;


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
void Shoot_Control(int16_t* downtriggeroutput,int16_t* uptriggeroutput);

#endif
