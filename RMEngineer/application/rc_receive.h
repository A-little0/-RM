/**
  ****************************(C) COPYRIGHT 2024 征途****************************
  * @file      rc_receive.c/h
  * @brief      
  *             这里是遥控器接收函数，通过dma空闲中断接收和处理遥控器数据.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Jan-29-2024    chenjiangnan    
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 征途****************************
  */
#ifndef __RC_RECEIVE_H
#define __RC_RECEIVE_H

#include "main.h"
#include "usart.h"
#include "dma.h"
#include "rc_task.h"
#include "gimbal_task.h"
#include "endeffector_task.h"

#define RC_sent_X 4    	//底盘速度灵敏度，遥控器的摇杆值经过转换，一般为 ±660，如果对于该数值有更高倍率的要求，可以通过此处放大
#define RC_sent_Y 4	   	//底盘速度灵敏度，遥控器的摇杆值经过转换，一般为 ±660，如果对于该数值有更高倍率的要求，可以通过此处放大
#define RC_sent_rotate 4//底盘速度灵敏度，遥控器的摇杆值经过转换，一般为 ±660，如果对于该数值有更高倍率的要求，可以通过此处放大
#define RC_sent_updown  1//云台升降机构速度灵敏度，遥控器的摇杆值经过转换，一般为 ±660，如果对于该数值有更高倍率的要求，可以通过此处放大
#define RC_sent_pushpull 2//云台伸缩机构速度灵敏度，遥控器的摇杆值经过转换，一般为 ±660，如果对于该数值有更高倍率的要求，可以通过此处放大
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
  * @brief          遥控器接收初始化函数
  * @param[in]      none
  * @retval         none
  */
void RC_Receive_Init(void);

/**
  * @func			void IDLE_RC_Handler(void)
  * @brief          遥控器接收空闲中断处理
  * @brief          该函数在stm32f4xx_it.c中被相应串口中断处理函数调用
  * @param[in]      none
  * @retval         none
  */
void RC_Receive(void);

/**
  * @func			void RC_Receive(void)
  * @brief          处理接收的遥控器数据包
  * @param[in]      none
  * @retval         none
  */
void IDLE_RC_Handler(void);

#endif

