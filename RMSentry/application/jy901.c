/**
  ****************************(C) COPYRIGHT 2024 ��;****************************
  * @file       jy901.c/h
  * @brief      
  *             ������jy901������������ͨ��dma���ڿ����жϽ��պʹ������ݰ�.
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
#include "jy901.h"
#include "stdio.h"

/*����jy901���*/

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
  * @brief          JY901���ճ�ʼ������
  * @param[in]      none
  * @retval         none
  */
void JY901_Init(void)
{
	/*jy901�����ʼ��*/	
	User_USART_Init(&JY901_data);
	/*����dma�����жϽ���*/
    __HAL_UART_CLEAR_IDLEFLAG(&huart6);
    __HAL_UART_ENABLE_IT(&huart6,UART_IT_IDLE);
    HAL_UART_Receive_DMA(&huart6,JY901_data.RxBuffer,RXBUFFER_LEN);
}

/**
  * @func			void IDLE_IMU_Handler(void)
  * @brief          JY901���տ����жϴ���
  * @brief          �ú�����stm32f4xx_it.c�б���Ӧ�����жϴ���������
  * @param[in]      none
  * @retval         none
  */

void IDLE_IMU_Handler(void)
{
	
	 uint32_t Data_lave,Data_exist; 
	if((__HAL_UART_GET_FLAG(&huart6,UART_FLAG_IDLE)!= RESET))//�жϴ���dma�����Ƿ��ڿ���״̬
	{
		__HAL_UART_CLEAR_IDLEFLAG(&huart6);//���DMA�����жϱ�־λ
		
		HAL_UART_DMAStop(&huart6); //ֹͣ����dma����
		
		Data_lave = __HAL_DMA_GET_COUNTER(&hdma_usart6_rx); //��ȡ���ճ���

		Data_exist = 11-Data_lave; //����ʣ�೤��

		if(Data_exist == 0)  //�ж��Ƿ���յ���ȷ��������
		{
			/*���ճɹ��󣬴�����Ӧ��������*/
			JY901_Process(&JY901_data);
			JY901_ExpandAngle(&JY901_data);
		}
		HAL_UART_Receive_DMA(&huart6,JY901_data.RxBuffer,RXBUFFER_LEN);//��������dma����
	}
}

/**
  * @func			void User_USART_Init(User_USART *Data)
  * @brief          JY901�����ʼ������
  * @param[in]      Data��JY901���
  * @retval         none
  */
void User_USART_Init(User_USART *Data)
{
	if(Data==NULL)
	{
		return;
	}
	/*��ʼ����������*/
	for(uint16_t i=0; i < RXBUFFER_LEN; i++)	Data->RxBuffer[i] = 0;
	/*��ʼ��֡ͷ*/
	Data->frame_head= 0x55;
	/*��ʼ����־λ*/
	Data->Rx_flag = 0;
	/*��ʼ�����ݰ�����*/
	Data->Rx_len = RXBUFFER_LEN;
}


/**
  * @func			void JY901_Process( User_USART* JY901_data)
  * @brief          JY901���ݰ���������
  * @param[in]      JY901_data��JY901���
  * @retval         none
  */
void JY901_Process( User_USART* JY901_data)
{
	if(JY901_data==NULL)
	{
		return;
	}
	if(JY901_data->Rx_len < RXBUFFER_LEN) return;   	//���λ������

	for(uint8_t i=0;i<RXBUFFER_LEN;i++)
	{
		if(JY901_data->RxBuffer[i*11]!= JY901_data->frame_head) return;	//���֡ͷ����
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
				
				case 0x54:	//�ų�����
					memcpy(&stcMag,&JY901_data->RxBuffer[2 + i*11],8);
					for(uint8_t j = 0; j < 3; j++) 
				JY901_data->h.h[j] = (float)stcMag.h[j];
				break;
				
				case 0x55:	//D0-D3�˿�״̬
				break;
				
				case 0x56:	//��ѹ�߶�
					memcpy(&stcPress,&JY901_data->RxBuffer[2 + i*11],8);			
				JY901_data->lPressure.lPressure = (float)stcPress.lPressure;
				JY901_data->lPressure.lAltitude = (float)stcPress.lAltitude/100;
				break;
				
				case 0x57:	//��γ��
					memcpy(&stcLonLat.lLat,&JY901_data->RxBuffer[2 + i*11],8);
				JY901_data->lLon.lLat = (float)stcLonLat.lLat/10000000+(double)(stcLonLat.lLat % 10000000)/1e5;	
				JY901_data->lLon.lLat = (float)stcLonLat.lLon/10000000+(double)(stcLonLat.lLon % 10000000)/1e5;
				break;
				
				case 0x58:	//GPS
				break;
				
				case 0x59:	//��Ԫ��
					memcpy(&stcQ,&JY901_data->RxBuffer[2 + i*11],8);
					for(uint8_t j = 0; j < 4; j++) 
				JY901_data->q.q[j] = (float)stcQ.q[j]/32768;		
				break;	
		}
	}
}

/**
  * @func			void JY901_ExpandAngle(User_USART* JY901_data)
  * @brief          ��չJY901�Ƕȷ�Χ
  * @param[in]      JY901_data��JY901���
  * @retval         none
  */
void JY901_ExpandAngle(User_USART* JY901_data)
{
	if(JY901_data==NULL)
	{
		return;
	}
	static float last_angle=0;
	static int temp=0;//�ж��Ƿ��ǵ�һ�ν���
	if(temp==0)
	{	
		last_angle=JY901_data->angle.angle[2];
		temp=1;
	}
	else
	{
		float d_angle=JY901_data->angle.angle[2]-last_angle;
		//printf("%f,%f,%f\n",JY901_data->angle.angle[2],last_angle,d_angle);
		/*�жϽǶ��Ƿ���ͻ��*/
		if(d_angle<-340||temp==2)//������+ͻ��
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
		else if(d_angle>340||temp==3)//������+ͻ��
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
		else//δͻ��
		{
			temp=1;
			JY901_data->expand_angle.angle[2]=JY901_data->angle.angle[2];
		}
		JY901_data->expand_angle.angle[0]=JY901_data->angle.angle[0];
		JY901_data->expand_angle.angle[1]=JY901_data->angle.angle[1];
		last_angle=JY901_data->angle.angle[2];
	}
}
