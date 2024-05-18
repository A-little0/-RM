/**
  ****************************(C) COPYRIGHT 2024 征途****************************
  * @file      pidControllerAnalysis.c/h
  * @brief      
  *            这里是针对pid控制器，分析二阶系统的性能指标程序:
  *			   1.上升时间:系统第一次到达稳定点的时间，体现是系统的响应速度
  *			   2.最大超调量:系统输出的最大值减去稳态值*100%
  *			   3.稳定时间:系统进入稳态的误差时间，一般为最终状态的2%
  *			   系统响应:
  *			   1.阶跃响应
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Feb-12-2024    chenjiangnan    
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 征途****************************
  */
  
 #include "pidControllerAnalysis.h"
 
 
 /**
  * @func			void SystemAnalysis_Init(SystemAnalysis_HandleTypedef* analysisdata)
  * @brief          初始化函数
  * @param[in]      analysisdata：系统分析数据句柄
  * @retval         none
  */
void SystemAnalysis_Init(SystemAnalysis_HandleTypedef* analysisdata)
{
	if (analysisdata == NULL)
	{
		return;
	}
	/*初始化指标相关变量*/
	analysisdata->rise_time = 0;
	analysisdata->max_overshoot = 0.0f;
	analysisdata->settling_time = 0;
	analysisdata->sys_score = 0.0f;
	/*初始化指标相关权重*/
	analysisdata->tr_score = RISE_TIME_SCORE;
	analysisdata->mp_score = MAX_OVERSHOOT_SCORE;
	analysisdata->ts_score = SETTLING_TIME_SCORE;
	//analysisdata->max_score=RISE_TIME_SCORE+MAX_OVERSHOOT_SCORE+SETTLING_TIME_SCORE;
}

 /**
  * @func			void SystemAnalysis_Task(float rev, float set, SystemAnalysis_HandleTypedef* analysisdata)
  * @brief          系统性能分析任务
  * @param[in]      rev：返回值
  * @param[in]      set：设定值
  * @param[in]      analysisdata：系统分析数据句柄
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
	/*第一次进入函数*/
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
	/*第n次进入函数*/
	if (last_set == now_set)
	{
		/*上升时间*/
		float error = now_set - now_rev;
		if (temp_key2 == 0 && (error > 0 ? error : -error) > (now_set - began_rev > 0 ? now_set - began_rev : began_rev - now_set) * (1 - RISE_TIME_RATE))
		{
			analysisdata->rise_time++;
		}
		else
		{
			temp_key2 = 1;
		}
		/*最大超调量*/
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
		/*稳定时间*/
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
