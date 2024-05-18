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
#include "rc_receive.h"

RC_Ctl_t RC_Ctl;   					//声明遥控器数据结构体
uint8_t sbus_rx_buffer[18]; 		//声明遥控器缓存数组
RC_USER rc_user;

/**
  * @func			void RC_Receive_Init(void)
  * @brief          遥控器接收初始化函数
  * @param[in]      none
  * @retval         none
  */
void RC_Receive_Init(void)
{
	/*开启dma空闲中断接收*/
	__HAL_UART_CLEAR_IDLEFLAG(&huart3);
	__HAL_UART_ENABLE_IT(&huart3,UART_IT_IDLE);
	HAL_UART_Receive_DMA(&huart3,sbus_rx_buffer,18);
	/*初始化rc_user各值*/
	rc_user.x=0;
    rc_user.y=0;
    rc_user.z=0;
	/*使yaw,pitch刚开始处于机械中值*/
   //rc_user.yaw=YAW_MEDIAN;
	// rc_user.pitch=PITCH_MEDIAN;
	/*rc_status*/
}
/**
  * @func			void IDLE_RC_Handler(void)
  * @brief          遥控器接收空闲中断处理
  * @brief          该函数在stm32f4xx_it.c中被相应串口中断处理函数调用
  * @param[in]      none
  * @retval         none
  */
void IDLE_RC_Handler(void)
{

	uint32_t Data_lave,Data_exist; 
	if((__HAL_UART_GET_FLAG(&huart3,UART_FLAG_IDLE)!= RESET))//判断串口dma接收是否处于空闲状态  
	{
		__HAL_UART_CLEAR_IDLEFLAG(&huart3);//清空空闲标志位
		
		HAL_UART_DMAStop(&huart3); //停止串口dma接收
		
		Data_lave = __HAL_DMA_GET_COUNTER(&hdma_usart3_rx); //获取接收长度

		Data_exist = 18-Data_lave; //计算剩余长度

		if(Data_exist == 0)  //判断是否接收到正确长度数据
		{
			RC_Receive();//接收成功后，处理相应接收数据
		}
		else
		{
			RC_Receive_Init();
		}
		HAL_UART_Receive_DMA(&huart3,sbus_rx_buffer,18);//重新启动dma接收
	}
}
/**
  * @func			void RC_Receive(void)
  * @brief          处理接收的遥控器数据包
  * @param[in]      none
  * @retval         none
  */
void RC_Receive(void)
{
		/*拆包*/
	    RC_Ctl.rc.ch0 = (sbus_rx_buffer[0]| (sbus_rx_buffer[1] << 8)) & 0x07ff;          
		RC_Ctl.rc.ch1 = ((sbus_rx_buffer[1] >> 3) | (sbus_rx_buffer[2] << 5)) & 0x07ff;       
		RC_Ctl.rc.ch2 = ((sbus_rx_buffer[2] >> 6) | (sbus_rx_buffer[3] << 2) | (sbus_rx_buffer[4] << 10)) & 0x07ff;          
		RC_Ctl.rc.ch3 = ((sbus_rx_buffer[4] >> 1) | (sbus_rx_buffer[5] << 7)) & 0x07ff;           
		RC_Ctl.rc.s1  = ((sbus_rx_buffer[5] >> 4)& 0x000C) >> 2;                           
		RC_Ctl.rc.s2  = ((sbus_rx_buffer[5] >> 4)& 0x0003);    
		RC_Ctl.rc.roller=((((int16_t)sbus_rx_buffer[16])|(int16_t)sbus_rx_buffer[17]<<8)&0x07FF);
		

		RC_Ctl.mouse.x = sbus_rx_buffer[6] | (sbus_rx_buffer[7] << 8);                    //!< Mouse X axis        
		RC_Ctl.mouse.y = sbus_rx_buffer[8] | (sbus_rx_buffer[9] << 8);                    //!< Mouse Y axis      
		RC_Ctl.mouse.z = sbus_rx_buffer[10] | (sbus_rx_buffer[11] << 8);                  //!< Mouse Z axis         
		RC_Ctl.mouse.press_l = sbus_rx_buffer[12];                                        //!< Mouse Left Is Press      
		RC_Ctl.mouse.press_r = sbus_rx_buffer[13];                                        //!< Mouse Right Is Press 
		RC_Ctl.key.v = sbus_rx_buffer[14] | (sbus_rx_buffer[15] << 8);   				  //!< KeyBoard value
		
		/*底盘控制模式切换*/
		switch(RC_Ctl.rc.s1)
		{
			case 1:
				/*底盘控制 以底盘坐标系为参考的通道映射 */
				rc_control_mode=CHASSIS_NORMAL_MODE;
				/*速度使用绝对值*/
				rc_user.x     =  ( RC_Ctl.rc.ch3-1024 ) * RC_sent_X;//通道2控制左右
				rc_user.y     =  ( RC_Ctl.rc.ch2-1024 ) * RC_sent_Y;//通道3控制前后
				rc_user.z     = -( RC_Ctl.rc.roller -1024) * RC_sent_rotate;//通道roller控制绕圆转动
			    /*角度使用增量*/
			    rc_user.updown  =   ( RC_Ctl.rc.ch0-1024 )* RC_sent_updown ;//通道0控制云台升降
			    rc_user.pushpull= 	( RC_Ctl.rc.ch1-1024 )* RC_sent_pushpull;//通道1控制云台伸缩
				break;
			case 3:
				/*底盘锁死 底盘不受遥控器控制*/
				rc_control_mode=CHASSIS_CONTROL_DISABLE_MODE;
				/*速度使用绝对值*/
				rc_user.x     =  ( RC_Ctl.rc.ch3-1024 ) * RC_sent_endeffector_pitch;//通道2控制左右
				rc_user.y     =  ( RC_Ctl.rc.ch2-1024 ) * RC_sent_endeffector_yaw;//通道3控制前后
				//roll轴电机为增量位置控制所以遥控器通道值不需要做增量处理
				rc_user.z     = -( RC_Ctl.rc.roller -1024) * RC_sent_endeffector_roll;//通道roller控制绕圆转			
			    /*角度使用增量*/
			    rc_user.updown  =   ( RC_Ctl.rc.ch0-1024 )* RC_sent_updown ;//通道0控制云台升降
			    rc_user.pushpull= 	( RC_Ctl.rc.ch1-1024 )* RC_sent_pushpull;//通道1控制云台伸缩
				break;
			case 2:
				rc_control_mode=CHASSIS_GYRO_MODE;
				/*速度使用绝对值*/
				rc_user.x     =  ( RC_Ctl.rc.ch3-1024 ) * RC_sent_X;//通道2控制左右
				rc_user.y     =  ( RC_Ctl.rc.ch2-1024 ) * RC_sent_Y;//通道3控制前后
			    rc_user.z     = -( RC_Ctl.rc.roller -1024) * RC_sent_rotate;//通道roller控制绕圆转动
			    /*角度使用增量*/
			    rc_user.updown  =   ( RC_Ctl.rc.ch0-1024 )* RC_sent_updown ;//通道0控制云台升降
			    rc_user.pushpull= 	( RC_Ctl.rc.ch1-1024 )* RC_sent_pushpull;//通道1控制云台伸缩
				break;
			default://未接收到遥控器的值
				rc_control_mode=RC_SIGNAL_UNLINK;
				RC_Receive_Init();
				break;
		}
		
		switch(RC_Ctl.rc.s2)
		{
			/*气泵通电*/
			case AIR_PUMP_POWER_ENABLE_MODE:
				break;
			/*气泵断电*/
			case AIR_PUMP_POWER_DISABLE_MODE:
				break;
			/*进气导通*/
			case SOLENOID_VALVE_ENABLE_MODE:
				break;
		}
		
}

