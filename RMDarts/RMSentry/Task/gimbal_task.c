/**
  ****************************(C) COPYRIGHT 2024 ��;****************************
  * @file       chassis_task.c/h
  * @brief      
  *             ��������̨������򣬰�����̨yaw���pitch��������Լ�pitch��������������
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Jan-29-2024    chenjiangnan    0.�������ļ�
  *  V1.0.1     Mar-5-2024     				   1.�޸ĽǶ�ͻ�䲶���߼�bug
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 ��;****************************
  */
#include "gimbal_task.h"

/*��̨�����ؾ��*/
M6020_HandleTypeDef   yaw_gimbal_motor;
M6020_HandleTypeDef pitch_gimbal_motor;

/*����������ز���*/
int compensation_factor;
float pitch_cos;
float pitch_sin;
/*��̨yaw����ز���*/
float yaw_beganangle;//��������Ϊ�ο�
int16_t yaw_beganecd;  //�Ա�����Ϊ�ο�
float yaw_median_expangle;//��������Ϊ�ο�
int yaw_key;
int yaw_key2;
int16_t d_ecd;
float d_angle;

/**
  * @func			void Gimbal_Init(void)
  * @brief          ��̨������ʼ��
  * @param[in]      none
  * @retval         none
  */
void Gimbal_Init(void)
{
	/*��ʼ����̨��ص��*/
	M6020_Init(2, &yaw_gimbal_motor,PID_POSITION,CAN_CMD_VOLTAGE);
	M6020_Init(1, &pitch_gimbal_motor,PID_POSITION,CAN_CMD_VOLTAGE);
	
	/*��ʼ��pitch����������ز���*/
	compensation_factor=4000;//4000
	pitch_cos=0;
	pitch_sin=0;
	
	/*��ʼ����̨yaw����ز���*/
	yaw_beganangle=0.0f;
	yaw_key=0;
	yaw_key2=0;
	d_ecd=0;
	d_angle=0;
}

/**
  * @func			int16_t Pitch_Gimbal_Task(int16_t targetpostion)
  * @brief          pitch�ᴮ��pid��������
  * @param[in]      targetpostion��pitch��Ŀ����Ա�����ֵ
  * @retval         pitch�ᴮ��pid���
  */
int16_t Pitch_Gimbal_Task(int16_t targetpostion)
{
	/*����pid*/
	M6020_GetBasicData(&pitch_gimbal_motor);//��ȡ����
	int16_t target_rpm=M6020_PositionLoopPIDController(&pitch_gimbal_motor.position_control_data,pitch_gimbal_motor.basic_data.ecd , targetpostion);//λ�û�
	int16_t output=M6020_SpeedLoopPIDController(&pitch_gimbal_motor.speed_control_data, pitch_gimbal_motor.basic_data.speed_rpm, target_rpm);//�ǶȻ�
	
	return output;
}

/**
  * @func			int16_t Yaw_Gimbal_Task(int16_t targetpostion)
  * @brief          yawh�ᴮ��pid��������
  * @brief          ��yawh����Ա�����Ϊ�ο�
  * @param[in]      targetpostion��yaw��Ŀ����Ա�����ֵ
  * @retval         yaw�ᴮ��pid���
  */
int16_t Yaw_Gimbal_Task(int16_t targetpostion)
{
	
	/*����pid*/
	M6020_GetBasicData(&yaw_gimbal_motor);//��ȡ����
	int16_t target_rpm=M6020_PositionLoopPIDController(&yaw_gimbal_motor.position_control_data,yaw_gimbal_motor.basic_data.ecd , targetpostion);//λ�û�
	int16_t output=M6020_SpeedLoopPIDController(&yaw_gimbal_motor.speed_control_data, yaw_gimbal_motor.basic_data.speed_rpm, target_rpm);//�ǶȻ�
	
	return output;
}

/**
  * @func			int16_t Yaw_Gimbal_Task2(float realangle ,float targetangle)
  * @brief          yawh�ᴮ��pid��������2
  * @brief          �������ǽǶ�(��������)Ϊ�ο�
  * @param[in]      realangle��yaw��ʵʱ�Ƕȣ���Χ[-180,180]
  * @param[in]      targetpostion��yaw��Ŀ��Ƕ�,��Χ[-180,180]
  * @retval         yaw�ᴮ��pid���
  */
int16_t Yaw_Gimbal_Task2(float realangle ,float targetangle)
{
	/*����Ŀ��Ƕ�����Ϊ-180~180��*/
	int num=0;
	if(targetangle>0)
	{
		num=(int)targetangle/180;
		if(num%2==0)
		{
			targetangle=targetangle-num*180;
		}
		else
		{
			targetangle=-180+(targetangle-num*180);
		}
	}
	else
	{
		num=(int)targetangle/180;
		if(num%2==0)
		{
			targetangle=targetangle-num*180;
		}
		else
		{
			targetangle=180+(targetangle-num*180);
		}
	}
	//printf("%f,%f\n",realangle,targetangle);	
	float rev=0.0f;
	float set=0.0f;
	/*����Ƕ�ͻ��*/
	if(targetangle>=0)//Ŀ��ֵ���ϰ�Բ
	{
		if(realangle>=0)//ʵ��ֵҲ���ϰ�Բ
		{
			rev=realangle;
			set=targetangle;
		}
		else//ʵ��ֵ���°�Բ
		{
			if(realangle>=targetangle-180)//Ŀ��ֵ��ʵ��ֵǰ��
			{
				rev=realangle;//rev=-realangle; //rev=realangle
				set=targetangle;//set=targetangle;//set=targetangle
			}
			else//Ŀ��ֵ��ʵ��ֵ����
			{
				rev=realangle+180;//rev=(180+realangle);   
				set=targetangle-180;//set=-(180-targetangle);set=targetangle-180
			}
		}
	}
	else//Ŀ��ֵ���°�Բ
	{
		if(realangle<0)//ʵ��ֵҲ���°�Բ
		{
			rev=realangle;
			set=targetangle;
		}
		else//ʵ��ֵ���ϰ�Բ
		{
			if(realangle<=targetangle+180)//ʵ��ֵ��Ŀ��ֵǰ��
			{
				rev=realangle;//rev=-realangle;//rev=realangle
				set=targetangle;//targetangle
			}
			else//ʵ��ֵ��Ŀ��ֵ����
			{
				rev=-(180-realangle);
				set=180+targetangle;
			}
		}
	}
	printf("%f,%f,%f\n",set,rev,set-rev);
	/*�Ƕ�ת���ɱ�����*/
	set=set*8191/360;
	rev=rev*8191/360;
	
	/*����pid*/
	int16_t target_rpm=M6020_PositionLoopPIDController(&yaw_gimbal_motor.position_control_data,rev, set);//λ�û�
	int16_t output=M6020_SpeedLoopPIDController(&yaw_gimbal_motor.speed_control_data, yaw_gimbal_motor.basic_data.speed_rpm, target_rpm);//�ǶȻ�
	
	return output;
}

/**
  * @func			float Update_ReferenceBeganAngle(User_USART* jy901_data)
  * @brief          ���²ο��Ƕ�
  * @brief          �������ǽǶ�(��������)Ϊ�ο�
  * @param[in]      jy901_data��jy901���
  * @retval         yaw����º�Ĳο��Ƕ�
  */
float Update_ReferenceBeganAngle(User_USART* jy901_data)
{
	yaw_beganangle=jy901_data->angle.angle[2];
	
	return yaw_beganangle;
}

/**
  * @func			float Get_ReferenceMedianAngle(User_USART* jy901_data,M6020_HandleTypeDef* device)
  * @brief          ����yaw���ڻ�е��ֵλ�õĽǶ�
  * @brief          �������ǽǶ�(��������)Ϊ�ο�
  * @param[in]      jy901_data��jy901���
  * @param[in]      device��6020������
  * @retval         yaw���ڻ�е��ֵλ�õĽǶ�
  */
float Get_ReferenceMedianAngle(User_USART* jy901_data,M6020_HandleTypeDef* device)
{
	/*��ȡ��ʼ�Ƕ�*/
	yaw_beganangle=jy901_data->angle.angle[2];
	/*����ƫ��Ƕ�*/
	d_ecd=device->basic_data.ecd-YAW_MEDIAN;
	d_angle=d_ecd*360/8191;
	/*�����е��ֵ�Ƕ�*/
	yaw_median_expangle=yaw_beganangle-d_angle;
	
	return yaw_median_expangle;
}

/**
  * @func			int16_t Update_ReferenceBeganeECD(M6020_HandleTypeDef* device)
  * @brief          ���²ο��Ƕ�
  * @brief          �Ծ��Ա���ֵΪ�ο�
  * @param[in]      device��6020������
  * @retval         yaw����º�Ĳο��Ƕ�(���Ա�����ֵ)
  */
int16_t Update_ReferenceBeganeECD(M6020_HandleTypeDef* device)
{
	yaw_beganecd=device->basic_data.ecd;
	
	return yaw_beganecd;
}

/**
  * @func			int16_t GravityCompensation(const M6020_HandleTypeDef* device)
  * @brief          pitch����������
  * @param[in]      device��6020������
  * @retval         ����ֵ
  */
int16_t GravityCompensation(const M6020_HandleTypeDef* device)
{
	float angle=(device->basic_data.ecd-PITCH_MEDIAN)*360/8191;
	float cos=0.0f;
	float sin=0.0f;
	arm_sin_cos_f32(angle,&sin,&cos);
	pitch_cos=cos;
	pitch_sin=sin;
	//printf("%f,%f,%f\n",angle,cos,compensation_factor*pitch_cos);
	
	return compensation_factor*pitch_cos;
}

/**
  * @func			void Printf_GimbalMessage(M6020_HandleTypeDef* device)
  * @brief          ��̨��������
  * @param[in]      targetangle������̨���Ŀ��λ��
  * @retval         none
  */
void Printf_GimbalMessage(M6020_HandleTypeDef* device)
{
	printf("%d,%d,%d,%d\n",(int)device->basic_data.ecd,(int)device->position_control_data.set,(int)device->basic_data.speed_rpm,(int)device->speed_control_data.set);
}
