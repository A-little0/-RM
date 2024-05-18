/**
  ****************************(C) COPYRIGHT 2024 ��;****************************
  * @file      pidControllerAnalysis.c/h
  * @brief      
  *            ���������pid����������������ϵͳ������ָ�����:
  *			   1.����ʱ��:ϵͳ��һ�ε����ȶ����ʱ�䣬������ϵͳ����Ӧ�ٶ�
  *			   2.��󳬵���:ϵͳ��������ֵ��ȥ��ֵ̬*100%
  *			   3.�ȶ�ʱ��:ϵͳ������̬�����ʱ�䣬һ��Ϊ����״̬��2%
  *			   ϵͳ��Ӧ:
  *			   1.��Ծ��Ӧ
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Feb-12-2024    chenjiangnan    
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 ��;****************************
  */
#ifndef __PIDCONTROLLERANALYSIS_H
#define __PIDCONTROLLERANALYSIS_H

#include "main.h"
/*����ʱ���*/
#define SAMPLING_TIME 5 //5ms

#define RISE_TIME_RATE 0.78f
#define SETTLING_TIME_RATE 0.95f
/*Ȩ�غ�*/
#define RISE_TIME_SCORE 4
#define MAX_OVERSHOOT_SCORE 3
#define SETTLING_TIME_SCORE 3

typedef struct{
	/*����ָ��*/
	uint16_t rise_time;//����ʱ��
	float max_overshoot;//��󳬵���
	uint16_t settling_time;//�ȶ�ʱ��
	/*ָ��Ȩ��*/
	uint8_t tr_score;
	uint8_t mp_score;
	uint8_t ts_score;
	/*ϵͳ���ܷ���*/
	float sys_score;//��������ȫ��ʵ��R
	//float max_score;
}SystemAnalysis_HandleTypedef;


 /**
  * @func			void SystemAnalysis_Init(SystemAnalysis_HandleTypedef* analysisdata)
  * @brief          ��ʼ������
  * @param[in]      analysisdata��ϵͳ�������ݾ��
  * @retval         none
  */
void SystemAnalysis_Init(SystemAnalysis_HandleTypedef* analysisdata);


 /**
  * @func			void SystemAnalysis_Task(float rev, float set, SystemAnalysis_HandleTypedef* analysisdata)
  * @brief          ϵͳ���ܷ�������
  * @param[in]      rev������ֵ
  * @param[in]      set���趨ֵ
  * @param[in]      analysisdata��ϵͳ�������ݾ��
  * @retval         none
  */
void SystemAnalysis_Task(float rev, float set, SystemAnalysis_HandleTypedef* analysisdata);
#endif
