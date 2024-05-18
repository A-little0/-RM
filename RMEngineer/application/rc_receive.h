/**
  ****************************(C) COPYRIGHT 2024 ��;****************************
  * @file      rc_receive.c/h
  * @brief      
  *             ������ң�������պ�����ͨ��dma�����жϽ��պʹ���ң��������.
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
#ifndef __RC_RECEIVE_H
#define __RC_RECEIVE_H

#include "main.h"
#include "usart.h"
#include "dma.h"
#include "rc_task.h"
#include "gimbal_task.h"
#include "endeffector_task.h"

#define RC_sent_X 4    	//�����ٶ������ȣ�ң������ҡ��ֵ����ת����һ��Ϊ ��660��������ڸ���ֵ�и��߱��ʵ�Ҫ�󣬿���ͨ���˴��Ŵ�
#define RC_sent_Y 4	   	//�����ٶ������ȣ�ң������ҡ��ֵ����ת����һ��Ϊ ��660��������ڸ���ֵ�и��߱��ʵ�Ҫ�󣬿���ͨ���˴��Ŵ�
#define RC_sent_rotate 4//�����ٶ������ȣ�ң������ҡ��ֵ����ת����һ��Ϊ ��660��������ڸ���ֵ�и��߱��ʵ�Ҫ�󣬿���ͨ���˴��Ŵ�
#define RC_sent_updown  1//��̨���������ٶ������ȣ�ң������ҡ��ֵ����ת����һ��Ϊ ��660��������ڸ���ֵ�и��߱��ʵ�Ҫ�󣬿���ͨ���˴��Ŵ�
#define RC_sent_pushpull 2//��̨���������ٶ������ȣ�ң������ҡ��ֵ����ת����һ��Ϊ ��660��������ڸ���ֵ�и��߱��ʵ�Ҫ�󣬿���ͨ���˴��Ŵ�
#define RC_sent_endeffector_roll  0.1
#define RC_sent_endeffector_yaw   0.05
#define RC_sent_endeffector_pitch 0.05

typedef struct
{
	struct
	{ 
		unsigned short ch0;
		unsigned short ch1;
		unsigned short ch2;
		unsigned short ch3;
		unsigned short roller;
		unsigned char s1;
		unsigned char s2;
	}rc;
	
	struct 
	{
		unsigned short x;
		unsigned short y;
		unsigned short z;
		unsigned char press_l;
		unsigned char press_r;
	}mouse;
	
	struct
	{
		unsigned short v;
	}key;
}RC_Ctl_t;
typedef struct
{ 
	signed short x;
	signed short y;
	signed short z;
	signed short updown;
	  signed short pushpull;
//	unsigned char s1;
//	unsigned char s2;
//	signed short status;
}RC_USER;

extern RC_Ctl_t RC_Ctl;   
extern RC_USER rc_user;

/**
  * @func			void RC_Receive_Init(void)
  * @brief          ң�������ճ�ʼ������
  * @param[in]      none
  * @retval         none
  */
void RC_Receive_Init(void);

/**
  * @func			void IDLE_RC_Handler(void)
  * @brief          ң�������տ����жϴ���
  * @brief          �ú�����stm32f4xx_it.c�б���Ӧ�����жϴ���������
  * @param[in]      none
  * @retval         none
  */
void RC_Receive(void);

/**
  * @func			void RC_Receive(void)
  * @brief          ������յ�ң�������ݰ�
  * @param[in]      none
  * @retval         none
  */
void IDLE_RC_Handler(void);

#endif

