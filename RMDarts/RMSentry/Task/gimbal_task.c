/**
  ****************************(C) COPYRIGHT 2024 征途****************************
  * @file       chassis_task.c/h
  * @brief      
  *             这里是云台任务程序，包含云台yaw轴和pitch轴的驱动以及pitch轴重力补偿程序
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Jan-29-2024    chenjiangnan    0.创建该文件
  *  V1.0.1     Mar-5-2024     				   1.修改角度突变捕获逻辑bug
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 征途****************************
  */
#include "gimbal_task.h"

/*云台电机相关句柄*/
M6020_HandleTypeDef   yaw_gimbal_motor;
M6020_HandleTypeDef pitch_gimbal_motor;

/*重力补偿相关参数*/
int compensation_factor;
float pitch_cos;
float pitch_sin;
/*云台yaw轴相关参数*/
float yaw_beganangle;//以陀螺仪为参考
int16_t yaw_beganecd;  //以编码器为参考
float yaw_median_expangle;//以陀螺仪为参考
int yaw_key;
int yaw_key2;
int16_t d_ecd;
float d_angle;

/**
  * @func			void Gimbal_Init(void)
  * @brief          云台驱动初始化
  * @param[in]      none
  * @retval         none
  */
void Gimbal_Init(void)
{
	/*初始化云台相关电机*/
	M6020_Init(2, &yaw_gimbal_motor,PID_POSITION,CAN_CMD_VOLTAGE);
	M6020_Init(1, &pitch_gimbal_motor,PID_POSITION,CAN_CMD_VOLTAGE);
	
	/*初始化pitch重力补偿相关参数*/
	compensation_factor=4000;//4000
	pitch_cos=0;
	pitch_sin=0;
	
	/*初始化云台yaw轴相关参数*/
	yaw_beganangle=0.0f;
	yaw_key=0;
	yaw_key2=0;
	d_ecd=0;
	d_angle=0;
}

/**
  * @func			int16_t Pitch_Gimbal_Task(int16_t targetpostion)
  * @brief          pitch轴串级pid控制任务
  * @param[in]      targetpostion：pitch轴目标绝对编码器值
  * @retval         pitch轴串级pid输出
  */
int16_t Pitch_Gimbal_Task(int16_t targetpostion)
{
	/*串级pid*/
	M6020_GetBasicData(&pitch_gimbal_motor);//获取数据
	int16_t target_rpm=M6020_PositionLoopPIDController(&pitch_gimbal_motor.position_control_data,pitch_gimbal_motor.basic_data.ecd , targetpostion);//位置环
	int16_t output=M6020_SpeedLoopPIDController(&pitch_gimbal_motor.speed_control_data, pitch_gimbal_motor.basic_data.speed_rpm, target_rpm);//角度环
	
	return output;
}

/**
  * @func			int16_t Yaw_Gimbal_Task(int16_t targetpostion)
  * @brief          yawh轴串级pid控制任务
  * @brief          以yawh轴绝对编码器为参考
  * @param[in]      targetpostion：yaw轴目标绝对编码器值
  * @retval         yaw轴串级pid输出
  */
int16_t Yaw_Gimbal_Task(int16_t targetpostion)
{
	
	/*串级pid*/
	M6020_GetBasicData(&yaw_gimbal_motor);//获取数据
	int16_t target_rpm=M6020_PositionLoopPIDController(&yaw_gimbal_motor.position_control_data,yaw_gimbal_motor.basic_data.ecd , targetpostion);//位置环
	int16_t output=M6020_SpeedLoopPIDController(&yaw_gimbal_motor.speed_control_data, yaw_gimbal_motor.basic_data.speed_rpm, target_rpm);//角度环
	
	return output;
}

/**
  * @func			int16_t Yaw_Gimbal_Task2(float realangle ,float targetangle)
  * @brief          yawh轴串级pid控制任务2
  * @brief          以陀螺仪角度(世界坐标)为参考
  * @param[in]      realangle：yaw轴实时角度，范围[-180,180]
  * @param[in]      targetpostion：yaw轴目标角度,范围[-180,180]
  * @retval         yaw轴串级pid输出
  */
int16_t Yaw_Gimbal_Task2(float realangle ,float targetangle)
{
	/*限制目标角度区间为-180~180度*/
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
	/*处理角度突变*/
	if(targetangle>=0)//目标值在上半圆
	{
		if(realangle>=0)//实际值也在上半圆
		{
			rev=realangle;
			set=targetangle;
		}
		else//实际值在下半圆
		{
			if(realangle>=targetangle-180)//目标值在实际值前面
			{
				rev=realangle;//rev=-realangle; //rev=realangle
				set=targetangle;//set=targetangle;//set=targetangle
			}
			else//目标值在实际值后面
			{
				rev=realangle+180;//rev=(180+realangle);   
				set=targetangle-180;//set=-(180-targetangle);set=targetangle-180
			}
		}
	}
	else//目标值在下半圆
	{
		if(realangle<0)//实际值也在下半圆
		{
			rev=realangle;
			set=targetangle;
		}
		else//实际值在上半圆
		{
			if(realangle<=targetangle+180)//实际值在目标值前面
			{
				rev=realangle;//rev=-realangle;//rev=realangle
				set=targetangle;//targetangle
			}
			else//实际值在目标值后面
			{
				rev=-(180-realangle);
				set=180+targetangle;
			}
		}
	}
	printf("%f,%f,%f\n",set,rev,set-rev);
	/*角度转换成编码器*/
	set=set*8191/360;
	rev=rev*8191/360;
	
	/*串级pid*/
	int16_t target_rpm=M6020_PositionLoopPIDController(&yaw_gimbal_motor.position_control_data,rev, set);//位置环
	int16_t output=M6020_SpeedLoopPIDController(&yaw_gimbal_motor.speed_control_data, yaw_gimbal_motor.basic_data.speed_rpm, target_rpm);//角度环
	
	return output;
}

/**
  * @func			float Update_ReferenceBeganAngle(User_USART* jy901_data)
  * @brief          更新参考角度
  * @brief          以陀螺仪角度(世界坐标)为参考
  * @param[in]      jy901_data：jy901句柄
  * @retval         yaw轴更新后的参考角度
  */
float Update_ReferenceBeganAngle(User_USART* jy901_data)
{
	yaw_beganangle=jy901_data->angle.angle[2];
	
	return yaw_beganangle;
}

/**
  * @func			float Get_ReferenceMedianAngle(User_USART* jy901_data,M6020_HandleTypeDef* device)
  * @brief          计算yaw轴在机械中值位置的角度
  * @brief          以陀螺仪角度(世界坐标)为参考
  * @param[in]      jy901_data：jy901句柄
  * @param[in]      device：6020电机句柄
  * @retval         yaw轴在机械中值位置的角度
  */
float Get_ReferenceMedianAngle(User_USART* jy901_data,M6020_HandleTypeDef* device)
{
	/*获取初始角度*/
	yaw_beganangle=jy901_data->angle.angle[2];
	/*计算偏差角度*/
	d_ecd=device->basic_data.ecd-YAW_MEDIAN;
	d_angle=d_ecd*360/8191;
	/*计算机械中值角度*/
	yaw_median_expangle=yaw_beganangle-d_angle;
	
	return yaw_median_expangle;
}

/**
  * @func			int16_t Update_ReferenceBeganeECD(M6020_HandleTypeDef* device)
  * @brief          更新参考角度
  * @brief          以绝对编码值为参考
  * @param[in]      device：6020电机句柄
  * @retval         yaw轴更新后的参考角度(绝对编码器值)
  */
int16_t Update_ReferenceBeganeECD(M6020_HandleTypeDef* device)
{
	yaw_beganecd=device->basic_data.ecd;
	
	return yaw_beganecd;
}

/**
  * @func			int16_t GravityCompensation(const M6020_HandleTypeDef* device)
  * @brief          pitch轴重力补偿
  * @param[in]      device：6020电机句柄
  * @retval         补偿值
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
  * @brief          云台调试任务
  * @param[in]      targetangle：各云台电机目标位置
  * @retval         none
  */
void Printf_GimbalMessage(M6020_HandleTypeDef* device)
{
	printf("%d,%d,%d,%d\n",(int)device->basic_data.ecd,(int)device->position_control_data.set,(int)device->basic_data.speed_rpm,(int)device->speed_control_data.set);
}
