/**
  ****************************(C) COPYRIGHT 2024 征途****************************
  * @file       jy901.c/h
  * @brief      
  *             这里是jy901的驱动函数，通过dma串口空闲中断接收和处理数据包.
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
#include "jy901.h"
#include "stdio.h"

/*定义jy901句柄*/

User_USART      JY901_data;


struct SAcc 	stcAcc;
struct SGyro 	stcGyro;
struct SAngle 	stcAngle;
struct SMag 	stcMag;
struct SPress 	stcPress;
struct SLonLat 	stcLonLat;
struct SQ stcQ;

/**
  * @func			void JY901_Init(void)
  * @brief          JY901接收初始化函数
  * @param[in]      none
  * @retval         none
  */
void JY901_Init(void)
{
	/*jy901句柄初始化*/	
	User_USART_Init(&JY901_data);
	/*开启dma空闲中断接收*/
    __HAL_UART_CLEAR_IDLEFLAG(&huart6);
    __HAL_UART_ENABLE_IT(&huart6,UART_IT_IDLE);
    HAL_UART_Receive_DMA(&huart6,JY901_data.RxBuffer,RXBUFFER_LEN);
}

/**
  * @func			void IDLE_IMU_Handler(void)
  * @brief          JY901接收空闲中断处理
  * @brief          该函数在stm32f4xx_it.c中被相应串口中断处理函数调用
  * @param[in]      none
  * @retval         none
  */

void IDLE_IMU_Handler(void)
{
	
	 uint32_t Data_lave,Data_exist; 
	if((__HAL_UART_GET_FLAG(&huart6,UART_FLAG_IDLE)!= RESET))//判断串口dma接收是否处于空闲状态
	{
		__HAL_UART_CLEAR_IDLEFLAG(&huart6);//清除DMA空闲中断标志位
		
		HAL_UART_DMAStop(&huart6); //停止串口dma接收
		
		Data_lave = __HAL_DMA_GET_COUNTER(&hdma_usart6_rx); //获取接收长度

		Data_exist = 11-Data_lave; //计算剩余长度

		if(Data_exist == 0)  //判断是否接收到正确长度数据
		{
			/*接收成功后，处理相应接收数据*/
			JY901_Process(&JY901_data);
			JY901_ExpandAngle(&JY901_data);
		}
		HAL_UART_Receive_DMA(&huart6,JY901_data.RxBuffer,RXBUFFER_LEN);//重新启动dma接收
	}
}

/**
  * @func			void User_USART_Init(User_USART *Data)
  * @brief          JY901句柄初始化函数
  * @param[in]      Data：JY901句柄
  * @retval         none
  */
void User_USART_Init(User_USART *Data)
{
	if(Data==NULL)
	{
		return;
	}
	/*初始化接收数组*/
	for(uint16_t i=0; i < RXBUFFER_LEN; i++)	Data->RxBuffer[i] = 0;
	/*初始化帧头*/
	Data->frame_head= 0x55;
	/*初始化标志位*/
	Data->Rx_flag = 0;
	/*初始化数据包长度*/
	Data->Rx_len = RXBUFFER_LEN;
}


/**
  * @func			void JY901_Process( User_USART* JY901_data)
  * @brief          JY901数据包解析函数
  * @param[in]      JY901_data：JY901句柄
  * @retval         none
  */
void JY901_Process( User_USART* JY901_data)
{
	if(JY901_data==NULL)
	{
		return;
	}
	if(JY901_data->Rx_len < RXBUFFER_LEN) return;   	//如果位数不对

	for(uint8_t i=0;i<RXBUFFER_LEN;i++)
	{
		if(JY901_data->RxBuffer[i*11]!= JY901_data->frame_head) return;	//如果帧头不对
		switch(JY901_data->RxBuffer[i*11+1])
		{
				case 0x51:	
					memcpy(&stcAcc,&JY901_data->RxBuffer[2 + i*11],8);
					for(uint8_t j = 0; j < 3; j++) 
				JY901_data->acc.a[j] = (float)stcAcc.a[j]/32768*16;
				break;
				
				case 0x52:	
					memcpy(&stcGyro,&JY901_data->RxBuffer[2 + i*11],8);
					for(uint8_t j = 0; j < 3; j++) 
				JY901_data->w.w[j] = (float)stcGyro.w[j]/32768*2000;
				break;
				
				case 0x53:			
					memcpy(&stcAngle,&JY901_data->RxBuffer[2 + i*11],8);
					for(uint8_t j = 0; j < 3; j++) 
				JY901_data->angle.angle[j] = (float)stcAngle.Angle[j]/32768*180;
				break;
				
				case 0x54:	//磁场解算
					memcpy(&stcMag,&JY901_data->RxBuffer[2 + i*11],8);
					for(uint8_t j = 0; j < 3; j++) 
				JY901_data->h.h[j] = (float)stcMag.h[j];
				break;
				
				case 0x55:	//D0-D3端口状态
				break;
				
				case 0x56:	//气压高度
					memcpy(&stcPress,&JY901_data->RxBuffer[2 + i*11],8);			
				JY901_data->lPressure.lPressure = (float)stcPress.lPressure;
				JY901_data->lPressure.lAltitude = (float)stcPress.lAltitude/100;
				break;
				
				case 0x57:	//经纬度
					memcpy(&stcLonLat.lLat,&JY901_data->RxBuffer[2 + i*11],8);
				JY901_data->lLon.lLat = (float)stcLonLat.lLat/10000000+(double)(stcLonLat.lLat % 10000000)/1e5;	
				JY901_data->lLon.lLat = (float)stcLonLat.lLon/10000000+(double)(stcLonLat.lLon % 10000000)/1e5;
				break;
				
				case 0x58:	//GPS
				break;
				
				case 0x59:	//四元数
					memcpy(&stcQ,&JY901_data->RxBuffer[2 + i*11],8);
					for(uint8_t j = 0; j < 4; j++) 
				JY901_data->q.q[j] = (float)stcQ.q[j]/32768;		
				break;	
		}
	}
}

/**
  * @func			void JY901_ExpandAngle(User_USART* JY901_data)
  * @brief          拓展JY901角度范围
  * @param[in]      JY901_data：JY901句柄
  * @retval         none
  */
void JY901_ExpandAngle(User_USART* JY901_data)
{
	if(JY901_data==NULL)
	{
		return;
	}
	static float last_angle=0;
	static int temp=0;//判断是否是第一次进入
	if(temp==0)
	{	
		last_angle=JY901_data->angle.angle[2];
		temp=1;
	}
	else
	{
		float d_angle=JY901_data->angle.angle[2]-last_angle;
		//printf("%f,%f,%f\n",JY901_data->angle.angle[2],last_angle,d_angle);
		/*判断角度是否发生突变*/
		if(d_angle<-340||temp==2)//正方向+突变
		{
			if(d_angle>340)
			{
				temp=1;
			}
			else
			{
				temp=2;
				JY901_data->expand_angle.angle[2]=360+JY901_data->angle.angle[2];
			}
			
		}
		else if(d_angle>340||temp==3)//负方向+突变
		{
			if(d_angle<-340)
			{
				temp=2;
			}
			else
			{
				temp=3;
				JY901_data->expand_angle.angle[2]=-360+JY901_data->angle.angle[2];
			}
		}
		else//未突变
		{
			temp=1;
			JY901_data->expand_angle.angle[2]=JY901_data->angle.angle[2];
		}
		JY901_data->expand_angle.angle[0]=JY901_data->angle.angle[0];
		JY901_data->expand_angle.angle[1]=JY901_data->angle.angle[1];
		last_angle=JY901_data->angle.angle[2];
	}
}
