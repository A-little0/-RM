/**
  ****************************(C) COPYRIGHT 2024 征途****************************
  * @file       2006_driver.c/h
  * @brief      
  *             这里是2006的驱动函数，通过pid控制控制电机.
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
#include "2006_driver.h"

//M2006_HandleTypeDef up_trigger_motor;
//M2006_HandleTypeDef down_trigger_motor;

/**
  * @func			void M2006_Init(uint8_t motorid,M2006_HandleTypeDef* device,enum PID_MODE pidmode)
  * @brief          2006电机相关参数初始化
  * @param[in]      motorid:初始化电机id
  * @param[in]      device:2006电机句柄
  * @param[in]      pidmode:pid的控制模式(增量式，位置式)
  * @retval         none
  */
void M2006_Init(uint8_t motorid,M2006_HandleTypeDef* device,enum PID_MODE pidmode)
{
	/*初始化电机相关参数*/
	device->motor_id=motorid;
	device->expand_ecd=0;
	device->cylinder_number=0;
	device->record_last_ecd=0;
//	for(int i=0;i< M2006_RECORD_EXP_ECDBOSX_LENGTH;i++){device->record_exp_ecdboxs[i]=0;}
	
	device->basic_data.ecd=0;
	device->basic_data.given_current=0;
	device->basic_data.last_ecd=0;
	device->basic_data.speed_rpm=0;
	device->basic_data.temperate=0;
	
	/*初始化pid相关参数*/
	fp32 pid_position[3]={0.0f};
	fp32 pid_speed[3]={0.0f};
    fp32 pid_current[3]={0.0f};
	fp32 integralvalve=0;
	if(pidmode==PID_POSITION)
	{
		switch(0x200+motorid)
		{
			case CAN_TRIGGER_DOMN_MOTOR_ID:		
			
			pid_speed[0]=DOWN2006_SPEEDPID_KP;
			pid_speed[1]=DOWN2006_SPEEDPID_KI;
			pid_speed[2]=DOWN2006_SPEEDPID_KD;
			
				break;
			case  CAN_TRIGGER_UP_MOTOR_ID:
			
			pid_speed[0]=UP2006_SPEEDPID_KP;
			pid_speed[1]=UP2006_SPEEDPID_KI;
			pid_speed[2]=UP2006_SPEEDPID_KD;
			
				break;
			default :
				break;
		}
	}
    PID_init(&(device->position_control_data),pidmode, pid_position, 16384,16384);
    PID_init(&(device->speed_control_data),   pidmode, pid_speed,    16384,16384);
	PID_init(&(device->currrent_control_data),pidmode, pid_current,  16384,16384);
}

/**
  * @func			void M2006_GetBasicData(M2006_HandleTypeDef* device)
  * @brief          获取2006电机相关参数
  * @param[in]      device:2006电机句柄
  * @retval         none
  */
void M2006_GetBasicData(M2006_HandleTypeDef* device)
{
	if(device==NULL)
	{
		return;
	}
	switch(device->motor_id+0x200)
	{
	  case CAN_TRIGGER_DOMN_MOTOR_ID:
      case CAN_TRIGGER_UP_MOTOR_ID:
			device->basic_data.ecd=get_trigger_motor_measure_point(device->motor_id-1)->ecd;
			device->basic_data.given_current=get_trigger_motor_measure_point(device->motor_id-1)->given_current;
			device->basic_data.last_ecd=get_trigger_motor_measure_point(device->motor_id-1)->last_ecd;
			device->basic_data.speed_rpm=get_trigger_motor_measure_point(device->motor_id-1)->speed_rpm;
			device->basic_data.temperate=get_trigger_motor_measure_point(device->motor_id-1)->temperate;
	  break;
		default:
			break;
			
	}
}

/**
  * @func			int16_t M2006_GetExpandECDData(M2006_HandleTypeDef* device)
  * @brief          获取2006电机编码器绝对角度(对编码器角度范围进行了扩充)
  * @brief          该函数对低速旋转的电机有效，高速旋转可能会存在丢圈问题
  * @param[in]      device:2006电机句柄
  * @retval         扩充后的编码器角度值
  */
int16_t M2006_GetExpandECDData(M2006_HandleTypeDef* device)
{
	if(device==NULL)
	{
		return 0;
	}
	//用上一次位置-这一次位置判断是否有突变
	int16_t d_ecd=device->basic_data.ecd-device->record_last_ecd;
	/*有突变记录一圈*/
	if(d_ecd>8000)//反转突变
	{
		device->cylinder_number--;
	}
	else if(d_ecd<-8000)//正转突变
	{
		device->cylinder_number++;
	}
	else//未突变
	{
		/*计算范围拓展后的绝对编码器值*/ 
		if(device->cylinder_number>=0)
		{
			device->expand_ecd=device->cylinder_number*8191+device->basic_data.ecd;
		}
		else
		{
			device->expand_ecd=device->cylinder_number*8191+8191-device->basic_data.ecd+8191;
		}                
	}
	/*更新上一次绝对编码器值*/
	device->record_last_ecd=device->basic_data.ecd;
	
	return device->expand_ecd;
}

/**
  * @func			float M2006_AbsoluteEncoderToAngle(int absoluteencoder)
  * @brief          将2006电机(转子)编码器绝对角度值 转换为 (减速箱)编码器绝对角度值
  * @param[in]      absoluteencoder:2006电机(转子)编码器绝对角度值
  * @retval         (减速箱)编码器绝对角度值
  */
float M2006_AbsoluteEncoderToAngle(int absoluteencoder)
{
	//转子角度转换
	float rev_angle=(float)absoluteencoder*360.0f/8191;
	//减速箱角度转换
	rev_angle/=M2006_REDUCTION_RATIO;
	
	return rev_angle;
}

//Locked-rotor detection
/**
  * @func			_userbool M2006_LockedMotorDetertion(M2006_HandleTypeDef* device)
  * @brief          2006电机堵转检测
  * @param[in]      device:2006电机句柄
  * @retval         bool值，user_true为电机堵转，否则为电机未堵转
  */
_userbool M2006_LockedMotorDetertion(M2006_HandleTypeDef* device)
{
	if(device==NULL)
	{
		return 0;
	}
	/*以堵转电流和目标速度为电机堵转判据*/
	if(absoulte_value(device->basic_data.given_current)>9000 && device->speed_control_data.set!=0)
	{
		//目标速度不为零，且电流值很大，判断为电机堵转
		return user_true;
	}
	else{
		return user_false;
	}
}

/**
  * @func			int16_t M2006_PositionLoopPIDController(pid_type_def *pid, fp32 ref, fp32 set)
  * @brief          2006位置环pid控制函数
  * @param[in]      pid:2006电机位置环pid句柄
  * @param[in]      ref:2006电机实际角度
  * @param[in]      set:2006电机目标角度
  * @retval         2006位置环pid输出值
  */
int16_t M2006_PositionLoopPIDController(pid_type_def *pid, fp32 ref, fp32 set)
{
	if (pid == NULL)
    {
        return 0.0f;
    }

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;
    pid->fdb = ref;
    pid->error[0] = set - ref;
    if (pid->mode == PID_POSITION)
    {
        pid->Pout = pid->Kp * pid->error[0];
	    pid->Iout += pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        LimitMax(pid->Iout, pid->max_iout);
        pid->out = pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
		
		return pid->out;
    }
	else
	{
		return 0;
	}
}

/**
  * @func			int16_t M2006_SpeedLoopPIDController(pid_type_def *pid, fp32 ref, fp32 set)
  * @brief          2006速度环pid控制函数
  * @param[in]      pid:2006电机速度环pid句柄
  * @param[in]      ref:2006电机实际转速
  * @param[in]      set:2006电机目标转速
  * @retval         2006速度环pid输出值
  */
int16_t M2006_SpeedLoopPIDController(pid_type_def *pid, fp32 ref, fp32 set)
{
	if (pid == NULL)
    {
        return 0.0f;
    }

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;
    pid->fdb = ref;
    pid->error[0] = set - ref;
    if (pid->mode == PID_POSITION)
    {
        pid->Pout = pid->Kp * pid->error[0];
        pid->Iout += pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        LimitMax(pid->Iout, pid->max_iout);
        pid->out = pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
		
		return pid->out;
    }
	else
	{
		return 0;
	}
}

/**
  * @func			int16_t M2006_CurrentLoopPIDController(pid_type_def *pid, fp32 ref, fp32 set)
  * @brief          2006电流环pid控制函数
  * @param[in]      pid:2006电机电流环pid句柄
  * @param[in]      ref:2006电机实际电流
  * @param[in]      set:2006电机目标电流
  * @retval         2006电流环pid输出值
  */
int16_t M2006_CurrentLoopPIDController(pid_type_def *pid, fp32 ref, fp32 set)
{
	if (pid == NULL)
    {
        return 0.0f;
    }

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;
    pid->fdb = ref;
    pid->error[0] = set - ref;
    if (pid->mode == PID_POSITION)
    {
        pid->Pout = pid->Kp * pid->error[0];
		pid->Iout += pid->Ki * pid->error[0];	
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        LimitMax(pid->Iout, pid->max_iout);
        pid->out = pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
		
		return pid->out;
    }
	else
	{
		return 0;
	}
}
