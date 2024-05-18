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
  
 #include "pidControllerAnalysis.h"
 
 
 /**
  * @func			void SystemAnalysis_Init(SystemAnalysis_HandleTypedef* analysisdata)
  * @brief          ��ʼ������
  * @param[in]      analysisdata��ϵͳ�������ݾ��
  * @retval         none
  */
void SystemAnalysis_Init(SystemAnalysis_HandleTypedef* analysisdata)
{
	if (analysisdata == NULL)
	{
		return;
	}
	/*��ʼ��ָ����ر���*/
	analysisdata->rise_time = 0;
	analysisdata->max_overshoot = 0.0f;
	analysisdata->settling_time = 0;
	analysisdata->sys_score = 0.0f;
	/*��ʼ��ָ�����Ȩ��*/
	analysisdata->tr_score = RISE_TIME_SCORE;
	analysisdata->mp_score = MAX_OVERSHOOT_SCORE;
	analysisdata->ts_score = SETTLING_TIME_SCORE;
	//analysisdata->max_score=RISE_TIME_SCORE+MAX_OVERSHOOT_SCORE+SETTLING_TIME_SCORE;
}

 /**
  * @func			void SystemAnalysis_Task(float rev, float set, SystemAnalysis_HandleTypedef* analysisdata)
  * @brief          ϵͳ���ܷ�������
  * @param[in]      rev������ֵ
  * @param[in]      set���趨ֵ
  * @param[in]      analysisdata��ϵͳ�������ݾ��
  * @retval         none
  */
void SystemAnalysis_Task(float rev, float set, SystemAnalysis_HandleTypedef* analysisdata)
{
	static float now_set = 0.0f;
	static float last_set = 0.0f;
	static float began_rev = 0.0f;
	static float now_rev = 0.0f;
	static float last_rev = 0.0f;
	static int temp_key = 0;
	static int temp_key2 = 0;
	now_set = set;
	now_rev = rev;
	/*��һ�ν��뺯��*/
	if (temp_key == 0)
	{
		SystemAnalysis_Init(analysisdata);
		began_rev = rev;
		analysisdata->rise_time = 1;
		analysisdata->max_overshoot = rev;
		temp_key = 1;
		temp_key2 = 0;
		last_set = now_set;
		last_rev = now_rev;

		return;
	}
	/*��n�ν��뺯��*/
	if (last_set == now_set)
	{
		/*����ʱ��*/
		float error = now_set - now_rev;
		if (temp_key2 == 0 && (error > 0 ? error : -error) > (now_set - began_rev > 0 ? now_set - began_rev : began_rev - now_set) * (1 - RISE_TIME_RATE))
		{
			analysisdata->rise_time++;
		}
		else
		{
			temp_key2 = 1;
		}
		/*��󳬵���*/
		if (now_set >= 0)
		{
			analysisdata->max_overshoot = (analysisdata->max_overshoot > now_rev ? analysisdata->max_overshoot : now_rev);
			analysisdata->max_overshoot-=now_set;
		}
		else
		{
			analysisdata->max_overshoot = (analysisdata->max_overshoot < now_rev ? analysisdata->max_overshoot : now_rev);
			analysisdata->max_overshoot-=now_set;
		}
		/*�ȶ�ʱ��*/
		if ((error > 0 ? error : -error) > (now_set - began_rev > 0 ? now_set - began_rev : began_rev - now_set) * (1 - SETTLING_TIME_RATE))
		{
			analysisdata->settling_time++;
		}
		else
		{
			int sum_score = analysisdata->tr_score + analysisdata->mp_score + analysisdata->ts_score;
			analysisdata->sys_score = -analysisdata->rise_time * analysisdata->tr_score / sum_score
									  -analysisdata->max_overshoot * analysisdata->mp_score / sum_score
									  -(float)analysisdata->settling_time * analysisdata->ts_score / sum_score;
		}

		last_set = now_set;
		last_rev = now_rev;
	}
	else
	{
		temp_key = 0;
	}

}
