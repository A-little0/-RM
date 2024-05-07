#include "ros_task.h"

ROS_DataTypeDef ros_data;

uint8_t ros_tx_messageboxs[ROS_TX_MESSAGEBOXS_LENGTH];
uint8_t ros_rx_messageboxs[ROS_RX_MESSAGEBOXS_LENGTH];

/*初始化函数*/
void ROS_TaskInit(void)
{
	//初始化发送数据
	for(uint8_t i=0;i<ROS_TX_MESSAGEBOXS_LENGTH;i++){ros_tx_messageboxs[i]=0;}
	//初始化接收数据
	for(uint8_t i=0;i<ROS_RX_MESSAGEBOXS_LENGTH;i++){ros_rx_messageboxs[i]=0;}
	//控制数据初始化
	ros_data.x=0;
	ros_data.y=0;
	ros_data.z=0;
}

/*发送函数*/
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

void ROS_Receive_Init(void)
{
	/*开启dma空闲中断接收*/
	__HAL_UART_CLEAR_IDLEFLAG(&huart1);
	__HAL_UART_ENABLE_IT(&huart1,UART_IT_IDLE);
	HAL_UART_Receive_DMA(&huart1,ros_rx_messageboxs,ROS_RX_MESSAGEBOXS_LENGTH);

}
/*接收函数*/
void ROS_ReceiveProcess(void)
{
	if(ros_rx_messageboxs[0] == 0x2c)//判断帧头
	{
		if(ros_rx_messageboxs[1] == 1)//未识别到装甲板
		{
			auto_aim.aim_signal=AUTO_AIM_GET;
		}
		else
		{
			auto_aim.aim_signal=AUTO_AIM_LOSE;
		}
		auto_aim.d_yaw=-(ros_rx_messageboxs[3]-120);//-(ros_rx_messageboxs[3]-120);
		auto_aim.d_pitch=(ros_rx_messageboxs[2]-160);
	}
}
void IDLE_ROS_Handler(void)
{
	uint32_t Data_lave,Data_exist; 
	if((__HAL_UART_GET_FLAG(&huart1,UART_FLAG_IDLE)!= RESET))//判断串口dma接收是否处于空闲状态  
	{
		__HAL_UART_CLEAR_IDLEFLAG(&huart1);//清空空闲标志位
		
		HAL_UART_DMAStop(&huart1); //停止串口dma接收
		
		Data_lave = __HAL_DMA_GET_COUNTER(&hdma_usart1_rx); //获取接收长度

		Data_exist = ROS_RX_MESSAGEBOXS_LENGTH-Data_lave; //计算剩余长度

		if(Data_exist == 0)  //判断是否接收到正确长度数据
		{
			ROS_ReceiveProcess();//接收成功后，处理相应接收数据
		}
		else
		{
			ROS_Receive_Init();
		}
		HAL_UART_Receive_DMA(&huart1,ros_rx_messageboxs,ROS_RX_MESSAGEBOXS_LENGTH);//重新启动dma接收
	}
}
