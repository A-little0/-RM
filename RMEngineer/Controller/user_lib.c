/**
  ****************************(C) COPYRIGHT 2024 ��;****************************
  * @file      user_lib.c/h
  * @brief      
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Jan-29-2024    chenjiangnan    
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 ��;****************************
  */
#include "user_lib.h"


/**
  * @func			int16_t absoulte_value(int16_t value)
  * @brief          ����ֵ����
  * @param[in]      value:Ҫ�����ֵ
  * @retval         ���ؾ���ֵ
  */
int16_t absoulte_value(int16_t value)
{
	return value > 0 ? value:-value;
}
