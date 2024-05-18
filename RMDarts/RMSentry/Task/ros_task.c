#include "ros_task.h"

uint8_t ros_tx_messageboxs[ROS_TX_MESSAGEBOXS_LENGTH];
uint8_t ros_rx_messageboxs[ROS_RX_MESSAGEBOXS_LENGTH];


void ROS_TransmitTask(int16_t speed_rpmMA,int16_t speed_rpmMB,int16_t speed_rpmMC,int16_t speed_rpmMD)
{
	int16_t average_speed_rpm_left= (speed_rpmMA+speed_rpmMD)/2;
	int16_t average_speed_rpm_right=(speed_rpmMB+speed_rpmMC)/2;
	ros_tx_messageboxs[0]=0x2c;
	ros_tx_messageboxs[1]=average_speed_rpm_left>>8;
	ros_tx_messageboxs[2]=average_speed_rpm_left;
	ros_tx_messageboxs[3]=average_speed_rpm_right>>8;
	ros_tx_messageboxs[4]=average_speed_rpm_right;
	ros_tx_messageboxs[5]=0x5c;
	
	HAL_UART_Transmit(&huart1,ros_tx_messageboxs,ROS_TX_MESSAGEBOXS_LENGTH,1000);
}

void ROS_ReceiveTask()
{
	
}
