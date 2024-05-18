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
#include "rc_receive.h"

RC_Ctl_t RC_Ctl;   					//����ң�������ݽṹ��
uint8_t sbus_rx_buffer[18]; 		//����ң������������
RC_USER rc_user;					//�����û�ң�����ṹ��
KeyBoard_USER keyboard_user;		//�����û�����ṹ��

int key_value_test=0;

/**
  * @func			void RC_Receive_Init(void)
  * @brief          ң�������ճ�ʼ������
  * @param[in]      none
  * @retval         none
  */
void RC_Receive_Init(void)
{
	/*����dma�����жϽ���*/
	__HAL_UART_CLEAR_IDLEFLAG(&huart3);
	__HAL_UART_ENABLE_IT(&huart3,UART_IT_IDLE);
	HAL_UART_Receive_DMA(&huart3,sbus_rx_buffer,18);
	/*��ʼ��rc_user��ֵ*/
	rc_user.x=0;
    rc_user.y=0;
    rc_user.z=0;
  //rc_user.yaw=YAW_MEDIAN;//ʹyaw����ֵ��ʼ�������ڽ���н���
	rc_user.pitch=PITCH_MEDIAN;//ʹpitch�տ�ʼ���ڻ�е��ֵ
	rc_user.status=0;//rc_status
	/*��ʼ��keyboard_user��ֵ*/
	keyboard_user.x=0;
	keyboard_user.y=0;
	keyboard_user.z=0;
	keyboard_user.yaw=0;//ʹyaw����ֵ��ʼ�������ڽ���н���
	keyboard_user.pitch=PITCH_MEDIAN;//ʹpitch�տ�ʼ���ڻ�е��ֵ
	
	/*������ģʽ��ʼ��Ϊң��������*/
	RC_Ctl.control_mode=RC_REMOTE_CONTROL_MODE;
}


/**
  * @func			void IDLE_RC_Handler(void)
  * @brief          ң�������տ����жϴ���
  * @brief          �ú�����stm32f4xx_it.c�б���Ӧ�����жϴ���������
  * @param[in]      none
  * @retval         none
  */
void IDLE_RC_Handler(void)
{
	uint32_t Data_lave,Data_exist; 
	if((__HAL_UART_GET_FLAG(&huart3,UART_FLAG_IDLE)!= RESET))//�жϴ���dma�����Ƿ��ڿ���״̬  
	{
		__HAL_UART_CLEAR_IDLEFLAG(&huart3);//��տ��б�־λ
		
		HAL_UART_DMAStop(&huart3); //ֹͣ����dma����
		
		Data_lave = __HAL_DMA_GET_COUNTER(&hdma_usart3_rx); //��ȡ���ճ���

		Data_exist = 18-Data_lave; //����ʣ�೤��

		if(Data_exist == 0)  //�ж��Ƿ���յ���ȷ��������
		{
			RC_Receive();//���ճɹ��󣬴�����Ӧ��������
		}
		else
		{
			RC_Receive_Init();
		}
		HAL_UART_Receive_DMA(&huart3,sbus_rx_buffer,18);//��������dma����
	}
}
/**
  * @func			void RC_Receive(void)
  * @brief          ������յ�ң�������ݰ�
  * @param[in]      none
  * @retval         none
  */

void RC_Receive(void)
{
		/*���*/
	    RC_Ctl.rc.ch0 = (sbus_rx_buffer[0]| (sbus_rx_buffer[1] << 8)) & 0x07ff;          
		RC_Ctl.rc.ch1 = ((sbus_rx_buffer[1] >> 3) | (sbus_rx_buffer[2] << 5)) & 0x07ff;       
		RC_Ctl.rc.ch2 = ((sbus_rx_buffer[2] >> 6) | (sbus_rx_buffer[3] << 2) | (sbus_rx_buffer[4] << 10)) & 0x07ff;          
		RC_Ctl.rc.ch3 = ((sbus_rx_buffer[4] >> 1) | (sbus_rx_buffer[5] << 7)) & 0x07ff;           
		RC_Ctl.rc.s1  = ((sbus_rx_buffer[5] >> 4)& 0x000C) >> 2;                           
		RC_Ctl.rc.s2  = ((sbus_rx_buffer[5] >> 4)& 0x0003);    
		RC_Ctl.rc.roller=((((int16_t)sbus_rx_buffer[16])|(int16_t)sbus_rx_buffer[17]<<8)&0x07FF);
		

		RC_Ctl.mouse.x = sbus_rx_buffer[6] | (sbus_rx_buffer[7] << 8);                  //!< Mouse X axis        
		RC_Ctl.mouse.y = sbus_rx_buffer[8] | (sbus_rx_buffer[9] << 8);                  //!< Mouse Y axis      
		RC_Ctl.mouse.z = sbus_rx_buffer[10]| (sbus_rx_buffer[11]<< 8);                  //!< Mouse Z axis         
		RC_Ctl.mouse.press_l = sbus_rx_buffer[12];                                      //!< Mouse Left Is Press      
		RC_Ctl.mouse.press_r = sbus_rx_buffer[13];                                      //!< Mouse Right Is Press 
		RC_Ctl.key.v = sbus_rx_buffer[14]; //| (sbus_rx_buffer[15] << 8);   		    //!< KeyBoard value
	
//	    if((RC_Ctl.key.v & KEY_PRESSED_OFFSET_E) != 0)
//		{
//			key_value_test++;
//		}
	    RC_Process();
		
		RC_Ctl.key.last_v=RC_Ctl.key.v;
}




