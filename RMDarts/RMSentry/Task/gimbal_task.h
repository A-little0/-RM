/**
  ****************************(C) COPYRIGHT 2024 ��;****************************
  * @file       chassis_task.c/h
  * @brief      
  *             ��������̨������򣬰�����̨yaw���pitch��������Լ�pitch��������������
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
#ifndef __GIMBAL_TASK_H
#define __GIMBAL_TASK_H

#include "6020_driver.h"
#include "arm_math.h"
#include "jy901.h"



#define YAW_MEDIAN 4092//yaw ���е��ֵ��������ֵ��
//#define YAW_MEDIAN 156.917725f// yaw ���е��ֵ

#define PITCH_MAXANGLE 4736//�������
#define PITCH_MEDIAN 3405//pitch ���е��ֵ����������
#define PITCH_MINANGLE 2620//��С������

/*��̨�����ؾ��*/
extern M6020_HandleTypeDef   yaw_gimbal_motor;
extern M6020_HandleTypeDef pitch_gimbal_motor;

/*����������ز���*/
extern int compensation_factor;
extern float pitch_cos;
extern float pitch_sin;
/*��̨yaw����ز���*/
extern float yaw_beganangle;
extern int16_t yaw_beganecd;  //�Ա�����Ϊ�ο�
extern int yaw_key;
extern int yaw_key2;

/**
  * @func			void Gimbal_Init(void)
  * @brief          ��̨������ʼ��
  * @param[in]      none
  * @retval         none
  */
void Gimbal_Init(void);


/**
  * @func			int16_t Pitch_Gimbal_Task(int16_t targetpostion)
  * @brief          pitch�ᴮ��pid��������
  * @param[in]      targetpostion��pitch��Ŀ����Ա�����ֵ
  * @retval         pitch�ᴮ��pid���
  */
int16_t Pitch_Gimbal_Task(int16_t targetpostion);


/**
  * @func			int16_t GravityCompensation(const M6020_HandleTypeDef* device)
  * @brief          pitch����������
  * @param[in]      device��6020������
  * @retval         ����ֵ
  */
int16_t GravityCompensation(const M6020_HandleTypeDef* device);



/**
  * @func			int16_t Yaw_Gimbal_Task(int16_t targetpostion)
  * @brief          yawh�ᴮ��pid��������
  * @brief          ��yawh����Ա�����Ϊ�ο�
  * @param[in]      targetpostion��yaw��Ŀ����Ա�����ֵ
  * @retval         yaw�ᴮ��pid���
  */
int16_t Yaw_Gimbal_Task(int16_t targetpostion);



/**
  * @func			int16_t Yaw_Gimbal_Task2(float realangle ,float targetangle)
  * @brief          yawh�ᴮ��pid��������2
  * @brief          �������ǽǶ�(��������)Ϊ�ο�
  * @param[in]      realangle��yaw��ʵʱ�Ƕȣ���Χ[-180,180]
  * @param[in]      targetpostion��yaw��Ŀ��Ƕ�,��Χ[-180,180]
  * @retval         yaw�ᴮ��pid���
  */
int16_t Yaw_Gimbal_Task2(float realangle ,float targetangle);


/**
  * @func			float Update_ReferenceBeganAngle(User_USART* jy901_data)
  * @brief          ���²ο��Ƕ�
  * @brief          �������ǽǶ�(��������)Ϊ�ο�
  * @param[in]      jy901_data��jy901���
  * @retval         yaw����º�Ĳο��Ƕ�
  */
float Update_ReferenceBeganAngle(User_USART* jy901_data);


/**
  * @func			float Get_ReferenceMedianAngle(User_USART* jy901_data,M6020_HandleTypeDef* device)
  * @brief          ����yaw���ڻ�е��ֵλ�õĽǶ�
  * @brief          �������ǽǶ�(��������)Ϊ�ο�
  * @param[in]      jy901_data��jy901���
  * @param[in]      device��6020������
  * @retval         yaw���ڻ�е��ֵλ�õĽǶ�
  */
float Get_ReferenceMedianAngle(User_USART* jy901_data,M6020_HandleTypeDef* device);


/**
  * @func			int16_t Update_ReferenceBeganeECD(M6020_HandleTypeDef* device)
  * @brief          ���²ο��Ƕ�
  * @brief          �Ծ��Ա���ֵΪ�ο�
  * @param[in]      device��6020������
  * @retval         yaw����º�Ĳο��Ƕ�(���Ա�����ֵ)
  */
int16_t Update_ReferenceBeganeECD(M6020_HandleTypeDef* device);


/**
  * @func			void Printf_GimbalMessage(M6020_HandleTypeDef* device)
  * @brief          ��̨��������
  * @param[in]      targetangle������̨���Ŀ��λ��
  * @retval         none
  */
void Printf_GimbalMessage(M6020_HandleTypeDef* device);

#endif
