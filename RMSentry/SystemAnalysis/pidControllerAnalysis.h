/**
  ****************************(C) COPYRIGHT 2024 征途****************************
  * @file      pidControllerAnalysis.c/h
  * @brief      
  *            这里是针对pid控制器，分析二阶系统的性能指标程序:
  *			   1.上升时间:系统第一次到达稳定点的时间，体现是系统的相应速度
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
#ifndef __PIDCONTROLLERANALYSIS_H
#define __PIDCONTROLLERANALYSIS_H

#include "main.h"
/*采样时间宏*/
#define SAMPLING_TIME 5 //5ms

#define RISE_TIME_RATE 0.78f
#define SETTLING_TIME_RATE 0.95f
/*权重宏*/
#define RISE_TIME_SCORE 4
#define MAX_OVERSHOOT_SCORE 3
#define SETTLING_TIME_SCORE 3

typedef struct{
	/*性能指标*/
	uint16_t rise_time;//上升时间
	float max_overshoot;//最大超调量
	uint16_t settling_time;//稳定时间
	/*指标权重*/
	uint8_t tr_score;
	uint8_t mp_score;
	uint8_t ts_score;
	/*系统性能分数*/
	float sys_score;//分数属于全体实数R
	//float max_score;
}SystemAnalysis_HandleTypedef;


 /**
  * @func			void SystemAnalysis_Init(SystemAnalysis_HandleTypedef* analysisdata)
  * @brief          初始化函数
  * @param[in]      analysisdata：系统分析数据句柄
  * @retval         none
  */
void SystemAnalysis_Init(SystemAnalysis_HandleTypedef* analysisdata);


 /**
  * @func			void SystemAnalysis_Task(float rev, float set, SystemAnalysis_HandleTypedef* analysisdata)
  * @brief          系统性能分析任务
  * @param[in]      rev：返回值
  * @param[in]      set：设定值
  * @param[in]      analysisdata：系统分析数据句柄
  * @retval         none
  */
void SystemAnalysis_Task(float rev, float set, SystemAnalysis_HandleTypedef* analysisdata);
#endif
