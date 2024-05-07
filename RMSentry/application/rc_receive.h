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
#include "detection_task.h"

/* ----------------------- RC Channel Definition---------------------------- */
#define RC_CH_VALUE_MIN ((uint16_t)364 )
#define RC_CH_VALUE_OFFSET ((uint16_t)1024)
#define RC_CH_VALUE_MAX ((uint16_t)1684)
/* ----------------------- RC Switch Definition----------------------------- */
#define RC_SW_UP ((uint16_t)1)
#define RC_SW_MID ((uint16_t)3)
#define RC_SW_DOWN ((uint16_t)2)
/* ----------------------- PC Key Definition-------------------------------- */
#define KEY_PRESSED_OFFSET_W ((uint16_t)0x01<<0)
#define KEY_PRESSED_OFFSET_S ((uint16_t)0x01<<1)
#define KEY_PRESSED_OFFSET_A ((uint16_t)0x01<<2)
#define KEY_PRESSED_OFFSET_D ((uint16_t)0x01<<3)
#define KEY_PRESSED_OFFSET_Q ((uint16_t)0x01<<6)//((uint16_t)0x01<<4)
#define KEY_PRESSED_OFFSET_E ((uint16_t)0x01<<7)//((uint16_t)0x01<<5)
#define KEY_PRESSED_OFFSET_SHIFT ((uint16_t)0x01<<4)//((uint16_t)0x01<<6)
#define KEY_PRESSED_OFFSET_CTRL  ((uint16_t)0x01<<5)//((uint16_t)0x01<<7)
#define RC_FRAME_LENGTH 18u

/* -----------------------Sensivity Definition Attention-------------------- */
//增量的灵敏度选择要注意增量运算的周期
//下面灵敏度的选择均在遥控器接收中断周期中确定的
/* ----------------------- RC Control Sensivity Definition-------------------- */
#define RC_sent_X 4    		//底盘速度灵敏度，遥控器的摇杆值经过转换，一般为 ±660，如果对于该数值有更高倍率的要求，可以通过此处放大
#define RC_sent_Y 4	   		//底盘速度灵敏度，遥控器的摇杆值经过转换，一般为 ±660，如果对于该数值有更高倍率的要求，可以通过此处放大
#define RC_sent_rotate 4	//底盘速度灵敏度，遥控器的摇杆值经过转换，一般为 ±660，如果对于该数值有更高倍率的要求，可以通过此处放大
#define RC_sent_yaw  0.01	//云台速度灵敏度，遥控器的摇杆值经过转换，一般为 ±660，如果对于该数值有更高倍率的要求，可以通过此处放大
#define RC_sent_pitch 0.1	//云台速度灵敏度，遥控器的摇杆值经过转换，一般为 ±660，如果对于该数值有更高倍率的要求，可以通过此处放大
/* ----------------------- key Board Control Sensivity Definition-------------------- */
#define KEY_SENT_CHASSIS_X 15
#define KEY_SENT_CHASSIS_Y 15
#define KEY_SENT_CHASSIS_Z 8
#define KEY_SENT_CHASSIS_ROTATE 1
#define KEY_SENT_GIMBAL_YAW 0.02
#define KEY_SENT_GIMBAL_PITCH 0.09
/* ----------------------- Control Mode Definition-------------------- */
#define  RC_REMOTE_CONTROL_MODE 0
#define  KEY_BOARD_CONTROL_MODE 1
typedef struct 
{
	struct
	{ 
		uint16_t ch0;
		uint16_t ch1;
		uint16_t ch2;
		uint16_t ch3;
		uint16_t roller;
		uint16_t s1;
		uint16_t s2;
	}rc;
	
	struct 
	{
		int16_t x;
		int16_t y;
		int16_t z;
		uint8_t press_l;
		uint8_t press_r;
	}mouse;
	
	struct
	{
		uint16_t v;
		uint16_t last_v;
	}key;
	
	uint8_t control_mode;
}RC_Ctl_t;

/*使用遥控器控制*/
typedef struct
{ 
	int16_t x;
	int16_t y;
	int16_t z;
	int16_t pitch;
	int16_t yaw;
	uint8_t last_s1;
	signed short status;
}RC_USER;
/*使用键鼠控制*/
typedef struct{
	/*底盘控制变量*/
	int16_t x;
	int16_t y;
	int16_t z;
	/*云台控制变量*/
	int16_t yaw;
	int16_t pitch;
	/*发射控制变量*/
	uint8_t leftshoot_status;
	uint8_t rightshoot_status;
	/*模拟s1*/
	uint8_t s1;
}KeyBoard_USER;



extern RC_Ctl_t RC_Ctl;   
extern RC_USER rc_user;
extern KeyBoard_USER keyboard_user;

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

