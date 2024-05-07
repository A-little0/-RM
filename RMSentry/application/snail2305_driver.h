#ifndef __SNAIL2305_DRIVER_H
#define __SNAIL2305_DRIVER_H

#include "tim.h"
#include "main.h"

/*��ದ�����*/
#define SNAIL_LEFT_UP_MAXPWM 1999
#define SNAIL_LEFT_UP_MINPWM 1160
#define SNAIL_LEFT_DOWN_MAXPWM 1999
#define SNAIL_LEFT_DOWN_MINPWM 1160
/*�Ҳದ�����*/
#define SNAIL_RIGHT_UP_MAXPWM 1993
#define SNAIL_RIGHT_UP_MINPWM 1100
#define SNAIL_RIGHT_DOWN_MAXPWM 1993
#define SNAIL_RIGHT_DOWN_MINPWM 1100
/*��ͣpwm*/
#define SNAIL_ALL_STOP_PWM 1000

typedef struct{
	uint16_t min_setup_pwm;
	uint16_t max_pwm;
}Snail2306_HandleTypeDef;

/**
  * @func			void Snail2305_Init(void)
  * @brief          ������Ӧ��ʱ����pwmͨ��
  * @param[in]      none
  * @retval         none
  */
void Snail2305_Init(void);

/**
  * @func			void SetLeft_Gun(int motorspeed)
  * @brief          ������ǹ��Ħ����ת��
  * @param[in]      motorspeed:snail2305���ת�ٿ���pwmռ�ձ�,��Χ[1160,2000]
  * @retval         none
  */
void SetLeft_Gun(int motorspeed);

/**
  * @func			void SetRight_Gun(int motorspeed)
  * @brief          ������ǹ��Ħ����ת��
  * @param[in]      motorspeed:snail2305���ת�ٿ���pwmռ�ձ�,��Χ[1160,2000]
  * @retval         none
  */
void SetRight_Gun(int motorspeed);
#endif
