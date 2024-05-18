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
 
#ifndef __USER_LIB_H
#define __USER_LIB_H

#include "main.h"

typedef int _userbool;
#define user_true   1
#define user_false -1

/**
  * @func			int16_t absoulte_value(int16_t value)
  * @brief          ����ֵ����
  * @param[in]      value:Ҫ�����ֵ
  * @retval         ���ؾ���ֵ
  */
int16_t absoulte_value(int16_t value);

#endif
