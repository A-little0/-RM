#ifndef __ROS_TASK_H
#define __ROS_TASK_H

#include "main.h"
#include "usart.h"
#include "autoaim_task.h"
#include "dma.h"

#define ROS_TX_MESSAGEBOXS_LENGTH 6
#define ROS_RX_MESSAGEBOXS_LENGTH 5


typedef struct{
	int16_t x;
	int16_t y;
	int16_t z;
}ROS_DataTypeDef;


extern ROS_DataTypeDef ros_data;
void ROS_TaskInit(void);
void ROS_TransmitTask(int16_t speed_rpmMA,int16_t speed_rpmMB,int16_t speed_rpmMC,int16_t speed_rpmMD);
void ROS_Receive_Init(void);
void ROS_ReceiveProcess(void);
void IDLE_ROS_Handler(void);
#endif
