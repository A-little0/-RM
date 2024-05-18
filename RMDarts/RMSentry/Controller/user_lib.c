/**
  ****************************(C) COPYRIGHT 2024 征途****************************
  * @file      user_lib.c/h
  * @brief      
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Jan-29-2024    chenjiangnan    1.添加滤波算法
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 征途****************************
  */
#include "user_lib.h"


/**
  * @func			int16_t absoulte_value(int16_t value)
  * @brief          绝对值函数
  * @param[in]      value:要处理的值
  * @retval         返回绝对值
  */
int16_t absoulte_value(int16_t value)
{
	return value > 0 ? value:-value;
}

int16_t Smoothing_filter(int16_t real_rpm,uint8_t* getlength,int16_t* rpm_boxs,uint8_t boxs_length)
{
		int16_t  filter_rpm=0;
		if(*getlength < boxs_length)
		{
			rpm_boxs[*getlength]=real_rpm;
			(*getlength)++;
		}

		for(int i=0;i<boxs_length-1;i++)
		{
			rpm_boxs[i]=rpm_boxs[i+1];
		}
		rpm_boxs[boxs_length-1]=real_rpm;
		for(int i =0;i<boxs_length;i++)
		{
			filter_rpm+=rpm_boxs[i];
		}
		filter_rpm=filter_rpm/boxs_length;
		
		return filter_rpm ;
}

float Smoothing_filter_f32(float real_rpm,uint8_t* getlength,float* rpm_boxs,uint8_t boxs_length)
{
		float  filter_rpm=0;
		if(*getlength < boxs_length)
		{
			rpm_boxs[*getlength]=real_rpm;
			(*getlength)++;
		}

		for(int i=0;i<boxs_length-1;i++)
		{
			rpm_boxs[i]=rpm_boxs[i+1];
		}
		rpm_boxs[boxs_length-1]=real_rpm;
		for(int i =0;i<boxs_length;i++)
		{
			filter_rpm+=rpm_boxs[i];
		}
		filter_rpm=filter_rpm/boxs_length;
		
		return filter_rpm ;
}
