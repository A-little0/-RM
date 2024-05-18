/**
  ****************************(C) COPYRIGHT 2024 ��;****************************
  * @file       chassis_task.c/h
  * @brief      
  *             ���������ֵ���������򣬰������ֽ��㣬����С���ݣ����̸�����̨���ܳ���
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
#ifndef __CHASSIS_TASK_H
#define __CHASSIS_TASK_H

#include "3508_driver.h"
#include "arm_math.h"
#include "user_lib.h"
/**/

/*С������ת�ٶ�*/
#define CHASSIS_GYRO_ROTATE_SPEED 3000
#define CHASSIS_MAX_XVECTOR_SPEED 2500
#define CHASSIS_MAX_YVECTOR_SPEED 2500
/*����pid��غ�*/
#define CHASSIS_ANGLEPID_KP 1.5f 
#define CHASSIS_ANGLEPID_KI 0.0f 
#define CHASSIS_ANGLEPID_KD 0.0f 

typedef struct{
	//���̽�ϵ
	int16_t c_x;//ָ�����ǰ��
	int16_t c_y;//ָ����̵���ǰ�����
	//����ӳ��-������
	int16_t c_rotate;//���������������µ���ת
	float32_t beganangle_cwtw;//����ӳ�������������µĳ�ʼƫ��
    float32_t angle_cmtw;//����ӳ�������������µ�ƫ��
	float32_t cos;
	float32_t sin;

}Chassis_CoordinateSystem_Typedef;

extern M3508_HandleTypeDef m3508_id1;//left front motor
extern M3508_HandleTypeDef m3508_id2;//right front motor
extern M3508_HandleTypeDef m3508_id3;//right rear motor
extern M3508_HandleTypeDef m3508_id4;//left rear motor

/**
  * @func			void Chassis_Init(void)
  * @brief          ����������ʼ��
  * @param[in]      none
  * @retval         none
  */
void Chassis_Init(void);


/**
  * @func			void Chassis_WheatWheel_Solution(
  *						int16_t chassisXvector,
  *						int16_t chassisYvectory,
  *						int16_t chassisRotatevector,
  *						int rotateK)
  * @brief          ���ֵ����˶�ѧ����
  * @brief          ��������ϵ(����ͼ)��
	/\����������			/\x(roll)
	||					||
	||					||
	||					||
		y(pitch)��========
		 
  * @param[in]      chassisXvector:��������ϵ�£�x�����µ��ٶ�
  * @param[in]      chassisYvectory:��������ϵ�£�y�����µ��ٶ�
  * @param[in]      chassisRotatevector:����(����)����ϵ�£�������ת���ٶ�
  * @param[in]      rotateK:��תϵ��(����̳�������й�k=a+b)
  * @retval         none
  */
void Chassis_WheatWheel_Solution(int16_t chassisXvector,int16_t chassisYvectory,int16_t chassisRotatevector,int rotateK);


/**
  * @func			void ChassisGyro_Task(
  *						int16_t worldXvector,
  *						int16_t worldYvector,
  *						int rotateK,
  *						float c_angle)
  * @brief          ����С��������
  * @param[in]      worldXvector������/��̨����ϵ�£�x�����ٶ�
  * @param[in]      worldYvector������/��̨����ϵ�£�y�����ٶ�
  * @param[in]      rotateK����תϵ��(����̳�������й�k=a+b)
  * @param[in]      c_angle������/��̨����ϵ���������ϵ�ļн�
  * @retval         none
  */
void ChassisGyro_Task(int16_t worldXvector,int16_t worldYvector,int rotateK,float c_angle);


/**
  * @func			void ChassisPositionControl_Task(
  *						int16_t worldXvector,
  *						int16_t worldYvector,
  *						int rotateK,
  *						float32_t realangle,
  *						float32_t targetangle)
  * @brief          ���̸�����̨����
  * @param[in]      worldXvector������/��̨����ϵ�£�x�����ٶ�
  * @param[in]      worldYvector������/��̨����ϵ�£�y�����ٶ�
  * @param[in]      rotateK����תϵ��(����̳�������й�k=a+b)
  * @param[in]      realangle��  ����/��̨����ϵ���������ϵ��ʵʱ�н�(yaw�������Ա�����ֵ)
  * @param[in]      targetangle������/��̨����ϵ���������ϵ��Ŀ��н�(yaw�������Ա�����ֵ)
  * @retval         none
  */
void ChassisPositionControl_Task(int16_t worldXvector,int16_t worldYvector,int rotateK,float32_t realangle,float32_t targetangle);


/**
  * @func			void Printf_ChassisMessage(int16_t targetspeed)
  * @brief          ���̵�������
  * @param[in]      targetangle�������̵��Ŀ���ٶ�
  * @retval         none
  */
void Printf_ChassisMessage(int16_t targetspeed);
#endif
