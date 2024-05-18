/**
  ****************************(C) COPYRIGHT 2024 征途****************************
  * @file       6020_driver.c/h
  * @brief      
  *             这里是6020的驱动函数，通过pid控制控制电机.
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
#include "6020_driver.h"

/**
  * @func			void M6020_Init(uint8_t motorid,M6020_HandleTypeDef* device,enum PID_MODE pidmode,enum CAN_CMD_MODE cmdmode)
  * @brief          6020电机相关参数初始化
  * @param[in]      motorid:初始化电机id
  * @param[in]      device:6020电机句柄
  * @param[in]      pidmode:pid的控制模式(增量式，位置式)
  * @param[in]      cmdmode:6020的控制模式( CAN_CMD_VOLTAGE：can发送电压控制信号，CAN_CMD_CURRENT：can发送电流控制信号)
  * @retval         none
  */
void M6020_Init(uint8_t motorid,M6020_HandleTypeDef* device,enum PID_MODE pidmode,enum CAN_CMD_MODE cmdmode)
{
	if(device==NULL)
	{
		return;
	}
	/*初始化电机相关参数*/
	device->motor_id=motorid;
	device->cmd_mode=cmdmode;
	
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
		switch(0x204+motorid)
		{
			case CAN_YAW_MOTOR_ID:
				
			pid_position[0]=YAWM6020_POSITIONPID_KP;
			pid_position[1]=YAWM6020_POSITIONPID_KI;
			pid_position[2]=YAWM6020_POSITIONPID_KP;
			
			pid_speed[0]=YAWM6020_SPEEDPID_KP;
			pid_speed[1]=YAWM6020_SPEEDPID_KI;
			pid_speed[2]=YAWM6020_SPEEDPID_KD;
			/*初始化积分分离阀门值*/
			integralvalve=YAW_INTERGRAL_VALVE;
				break;
			case  CAN_PIT_MOTOR_ID:
				
			pid_position[0]=PITCHM6020_POSITIONPID_KP;
			pid_position[1]=PITCHM6020_POSITIONPID_KI;
			pid_position[2]=PITCHM6020_POSITIONPID_KD;
			
			pid_speed[0]=PITCHM6020_SPEEDPID_KP;
			pid_speed[1]=PITCHM6020_SPEEDPID_KI;
			pid_speed[2]=PITCHM6020_SPEEDPID_KD;
			/*初始化积分分离阀门值*/
			integralvalve=PITCH_INTERGRAL_VALVE;
				break;
			default :
				break;
		}
	}
	
	if(cmdmode==CAN_CMD_CURRENT)
	{
		PID_init(&(device->position_control_data),pidmode, pid_position, 350,350);
		PID_init(&(device->speed_control_data),   pidmode, pid_speed,    16384,16384);
	}
	else if(cmdmode==CAN_CMD_VOLTAGE)
	{
		PID_init(&(device->position_control_data),pidmode, pid_position, 350,350);
		PID_init(&(device->speed_control_data),   pidmode, pid_speed,    25000,25000);
	}
	PID_init(&(device->current_control_data), pidmode, pid_current,  16384,16384);
	device->position_control_data.integral_valve=integralvalve;
	
}

/**
  * @func			void M6020_GetBasicData(M6020_HandleTypeDef* device)
  * @brief          获取6020电机相关参数
  * @param[in]      device:6020电机句柄
  * @retval         none
  */
void M6020_GetBasicData(M6020_HandleTypeDef* device)
{
	if(device==NULL)
	{
		return;
	}
	/*电机id判断*/
	switch(device->motor_id+0x204)
	{
	  case CAN_YAW_MOTOR_ID:
		device->basic_data.ecd=get_yaw_gimbal_motor_measure_point()->ecd;
		device->basic_data.given_current=get_yaw_gimbal_motor_measure_point()->given_current;
		device->basic_data.last_ecd=get_yaw_gimbal_motor_measure_point()->last_ecd;
		device->basic_data.speed_rpm=get_yaw_gimbal_motor_measure_point()->speed_rpm;
		device->basic_data.temperate=get_yaw_gimbal_motor_measure_point()->temperate;
	  break;
      case CAN_PIT_MOTOR_ID:
		device->basic_data.ecd=get_pitch_gimbal_motor_measure_point()->ecd;
		device->basic_data.given_current=get_pitch_gimbal_motor_measure_point()->given_current;
		device->basic_data.last_ecd=get_pitch_gimbal_motor_measure_point()->last_ecd;
		device->basic_data.speed_rpm=get_pitch_gimbal_motor_measure_point()->speed_rpm;
		device->basic_data.temperate=get_pitch_gimbal_motor_measure_point()->temperate;
	  break;
		default:
			break;
			
	}
}

/**
  * @func			int16_t M6020_PositionLoopPIDController(pid_type_def *pid, fp32 ref, fp32 set)
  * @brief          6020位置环pid控制函数
  * @param[in]      pid:6020电机位置环pid句柄
  * @param[in]      ref:6020电机实际角度
  * @param[in]      set:6020电机目标角度
  * @retval         6020位置环pid输出值
  */
int16_t M6020_PositionLoopPIDController(pid_type_def *pid, fp32 ref, fp32 set)
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
		//积分分离
		if(absoulte_value(pid->error[0])<pid->integral_valve)
		{
			pid->Iout += pid->Ki * pid->error[0];
		}
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
		//积分限幅
        LimitMax(pid->Iout, pid->max_iout);
        pid->out = pid->Pout + pid->Iout + pid->Dout;
		//输出限幅
        LimitMax(pid->out, pid->max_out);
		
		return pid->out;
    }
	else
	{
		return 0;
	}
}

/**
  * @func			int16_t M6020_SpeedLoopPIDController(pid_type_def *pid, fp32 ref, fp32 set)
  * @brief          2006速度环pid控制函数
  * @param[in]      pid:6020电机速度环pid句柄
  * @param[in]      ref:6020电机实际转速
  * @param[in]      set:6020电机目标转速
  * @retval         6020速度环pid输出值
  */
int16_t M6020_SpeedLoopPIDController(pid_type_def *pid, fp32 ref, fp32 set)
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
  * @func			int16_t M6020_CurrentLoopPIDController(pid_type_def *pid, fp32 ref, fp32 set)
  * @brief          6020电流环pid控制函数
  * @param[in]      pid:6020电机电流环pid句柄
  * @param[in]      ref:6020电机实际电流
  * @param[in]      set:6020电机目标电流
  * @retval         6020电流环pid输出值
  */
int16_t M6020_CurrentLoopPIDController(pid_type_def *pid, fp32 ref, fp32 set)
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


	
