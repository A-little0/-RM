/**
  ****************************(C) COPYRIGHT 2024 ��;****************************
  * @file       snail2305_driver.c/h
  * @brief      
  *             ������snail2305������������ͨ��pwm���ƿ��Ƶ��.
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
#include "snail2305_driver.h"


/**
  * @func			void Snail2305_Init(void)
  * @brief          ������Ӧ��ʱ����pwmͨ��
  * @param[in]      none
  * @retval         none
  */
void Snail2305_Init(void)
{
  HAL_TIM_Base_Start(&htim1);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);//1160~2000
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);//1160~2000
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);//1100~1993
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);//1100~1993
	
  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,1010);
  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,1010);
  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,1010);
  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,1010);
}

/**
  * @func			void SetLeft_Gun(int motorspeed)
  * @brief          ������ǹ��Ħ����ת��
  * @param[in]      motorspeed:snail2305���ת�ٿ���pwmռ�ձ�,��Χ[1160,2000]
  * @retval         none
  */
void SetLeft_Gun(int motorspeed)
{
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,motorspeed);
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,motorspeed);
}

/**
  * @func			void SetRight_Gun(int motorspeed)
  * @brief          ������ǹ��Ħ����ת��
  * @param[in]      motorspeed:snail2305���ת�ٿ���pwmռ�ձ�,��Χ[1160,2000]
  * @retval         none
  */
void SetRight_Gun(int motorspeed)
{
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,motorspeed);
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,motorspeed);
}
