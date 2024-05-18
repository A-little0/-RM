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
//������������ѡ��Ҫע���������������
//���������ȵ�ѡ�����ң���������ж�������ȷ����
/* ----------------------- RC Control Sensivity Definition-------------------- */
#define RC_sent_X 4    		//�����ٶ������ȣ�ң������ҡ��ֵ����ת����һ��Ϊ ��660��������ڸ���ֵ�и��߱��ʵ�Ҫ�󣬿���ͨ���˴��Ŵ�
#define RC_sent_Y 4	   		//�����ٶ������ȣ�ң������ҡ��ֵ����ת����һ��Ϊ ��660��������ڸ���ֵ�и��߱��ʵ�Ҫ�󣬿���ͨ���˴��Ŵ�
#define RC_sent_rotate 4	//�����ٶ������ȣ�ң������ҡ��ֵ����ת����һ��Ϊ ��660��������ڸ���ֵ�и��߱��ʵ�Ҫ�󣬿���ͨ���˴��Ŵ�
#define RC_sent_yaw  0.01	//��̨�ٶ������ȣ�ң������ҡ��ֵ����ת����һ��Ϊ ��660��������ڸ���ֵ�и��߱��ʵ�Ҫ�󣬿���ͨ���˴��Ŵ�
#define RC_sent_pitch 0.1	//��̨�ٶ������ȣ�ң������ҡ��ֵ����ת����һ��Ϊ ��660��������ڸ���ֵ�и��߱��ʵ�Ҫ�󣬿���ͨ���˴��Ŵ�
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

/*ʹ��ң��������*/
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
/*ʹ�ü������*/
typedef struct{
	/*���̿��Ʊ���*/
	int16_t x;
	int16_t y;
	int16_t z;
	/*��̨���Ʊ���*/
	int16_t yaw;
	int16_t pitch;
	/*������Ʊ���*/
	uint8_t leftshoot_status;
	uint8_t rightshoot_status;
	/*ģ��s1*/
	uint8_t s1;
}KeyBoard_USER;



extern RC_Ctl_t RC_Ctl;   
extern RC_USER rc_user;
extern KeyBoard_USER keyboard_user;

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

