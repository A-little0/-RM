/**
  ****************************(C) COPYRIGHT 2024 征途****************************
  * @file       3508_driver.c/h
  * @brief      
  *             这里是3508的驱动函数，通过pid控制控制电机.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Jan-29-2024    chenjiangnan    
  *                                             1.添加增量式PID函数API
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 征途****************************
  */
#include "3508_driver.h"

/**
  * @func			void M3508_Init(uint8_t motorid,M3508_HandleTypeDef* device,enum PID_MODE mode)
  * @brief          3508电机相关参数初始化
  * @param[in]      motorid:初始化电机id
  * @param[in]      device:3508电机句柄
  * @param[in]      pidmode:pid的控制模式(增量式，位置式)
  * @retval         none
  */
void M3508_Init(uint8_t motorid,M3508_HandleTypeDef* device,enum PID_MODE mode)
{
	//初始化电机id
	device->motor_id=motorid;
	//电机基础信息初始化
	device->basic_data.ecd=0;
	device->basic_data.given_current=0;
	device->basic_data.last_ecd=0;
	device->basic_data.speed_rpm=0;
	device->basic_data.temperate=0;
	//初始化pid
	fp32 PID_SPEED[3]={0};//速度环pid参数
	fp32 PID_CURRENT[3]={0};//电流环pid参数
	
	if(mode==PID_POSITION)
	{
		switch(0x200+motorid)
		{
			case CAN_3508_M1_ID:
			PID_SPEED[0]=M3508_ID1_SPEEDPID_POSITION_KP;
			PID_SPEED[1]=M3508_ID1_SPEEDPID_POSITION_KI;
			PID_SPEED[2]=M3508_ID1_SPEEDPID_POSITION_KD;
				break;
			case CAN_3508_M2_ID:
			PID_SPEED[0]=M3508_ID2_SPEEDPID_POSITION_KP;
			PID_SPEED[1]=M3508_ID2_SPEEDPID_POSITION_KI;
			PID_SPEED[2]=M3508_ID2_SPEEDPID_POSITION_KD;
				break;
			case CAN_3508_M3_ID:
			PID_SPEED[0]=M3508_ID3_SPEEDPID_POSITION_KP;
			PID_SPEED[1]=M3508_ID3_SPEEDPID_POSITION_KI;
			PID_SPEED[2]=M3508_ID3_SPEEDPID_POSITION_KD;
				break;
			case CAN_3508_M4_ID:
			PID_SPEED[0]=M3508_ID4_SPEEDPID_POSITION_KP;
			PID_SPEED[1]=M3508_ID4_SPEEDPID_POSITION_KI;
			PID_SPEED[2]=M3508_ID4_SPEEDPID_POSITION_KD;
				break;
			default :
				break;
		}
	}
	else if(mode == PID_DELTA)
	{
		switch(0x200+motorid)
		{
			case CAN_3508_M1_ID:
			PID_SPEED[0]=M3508_ID1_SPEEDPID_DELTA_KP;
			PID_SPEED[1]=M3508_ID1_SPEEDPID_DELTA_KI;
			PID_SPEED[2]=M3508_ID1_SPEEDPID_DELTA_KD;
				break;
			case CAN_3508_M2_ID:
			PID_SPEED[0]=M3508_ID2_SPEEDPID_DELTA_KP;
			PID_SPEED[1]=M3508_ID2_SPEEDPID_DELTA_KI;
			PID_SPEED[2]=M3508_ID2_SPEEDPID_DELTA_KD;
				break;
			case CAN_3508_M3_ID:
			PID_SPEED[0]=M3508_ID3_SPEEDPID_DELTA_KP;
			PID_SPEED[1]=M3508_ID3_SPEEDPID_DELTA_KI;
			PID_SPEED[2]=M3508_ID3_SPEEDPID_DELTA_KD;
				break;
			case CAN_3508_M4_ID:
			PID_SPEED[0]=M3508_ID4_SPEEDPID_DELTA_KP;
			PID_SPEED[1]=M3508_ID4_SPEEDPID_DELTA_KI;
			PID_SPEED[2]=M3508_ID4_SPEEDPID_DELTA_KD;
				break;
			case CAN_FRITION_LEFT_MOTOR_ID:
			PID_SPEED[0]=M3508_FRITION_LEFT_SPEEDPID_DELTA_KP;
			PID_SPEED[1]=M3508_FRITION_LEFT_SPEEDPID_DELTA_KI;
			PID_SPEED[2]=M3508_FRITION_LEFT_SPEEDPID_DELTA_KD;
				break;
			case CAN_FRITION_RIGHT_MOTOR_ID:
			PID_SPEED[0]=M3508_FRITION_RIGHT_SPEEDPID_DELTA_KP;
			PID_SPEED[1]=M3508_FRITION_RIGHT_SPEEDPID_DELTA_KI;
			PID_SPEED[2]=M3508_FRITION_RIGHT_SPEEDPID_DELTA_KD;
				break;
			case CAN_TRIGGER_ID:
			PID_SPEED[0]=M3508_TRIGGER_SPEEDPID_DELTA_KP;
			PID_SPEED[1]=M3508_TRIGGER_SPEEDPID_DELTA_KI;
			PID_SPEED[2]=M3508_TRIGGER_SPEEDPID_DELTA_KD;				
				break;
			default :
				break;
		}		
	}
	PID_init(&(device->speed_control_data), mode, PID_SPEED, 16384,16384);
	PID_init(&(device->currrent_control_data), mode, PID_CURRENT, 16384,16384);
	
}

/**
  * @func			void M3508_GetBasicData(M3508_HandleTypeDef* device)
  * @brief          获取3508电机相关参数
  * @param[in]      device:3508电机句柄
  * @retval         none
  */
void M3508_GetBasicData(M3508_HandleTypeDef* device)
{
	if(device==NULL)
	{
		return;
	}
	/*电机id判读*/
	switch(device->motor_id+0x200)
	{
		case CAN_3508_M1_ID:
        case CAN_3508_M2_ID:
        case CAN_3508_M3_ID:
        case CAN_3508_M4_ID: 
		device->basic_data.ecd=get_chassis_motor_measure_point(device->motor_id-1)->ecd;
		device->basic_data.given_current=get_chassis_motor_measure_point(device->motor_id-1)->given_current;
		device->basic_data.last_ecd=get_chassis_motor_measure_point(device->motor_id-1)->last_ecd;
		device->basic_data.speed_rpm=get_chassis_motor_measure_point(device->motor_id-1)->speed_rpm;
		device->basic_data.temperate=get_chassis_motor_measure_point(device->motor_id-1)->temperate;
		break;
		case CAN_FRITION_LEFT_MOTOR_ID:
		case CAN_FRITION_RIGHT_MOTOR_ID:
		device->basic_data.ecd=get_trigger_motor_measure_point(device->motor_id)->ecd;
		device->basic_data.given_current=get_trigger_motor_measure_point(device->motor_id)->given_current;
		device->basic_data.last_ecd=get_trigger_motor_measure_point(device->motor_id)->last_ecd;
		device->basic_data.speed_rpm=get_trigger_motor_measure_point(device->motor_id)->speed_rpm;
		device->basic_data.temperate=get_trigger_motor_measure_point(device->motor_id)->temperate;	
		break;
        case CAN_TRIGGER_ID:		
		device->basic_data.ecd=get_trigger3_motor_measure_point()->ecd;
		device->basic_data.given_current=get_trigger3_motor_measure_point()->given_current;
		device->basic_data.last_ecd=get_trigger3_motor_measure_point()->last_ecd;
		device->basic_data.speed_rpm=get_trigger3_motor_measure_point()->speed_rpm;
		device->basic_data.temperate=get_trigger3_motor_measure_point()->temperate;	
		break;
		default:
			break;
			
	}
}

/**
  * @func			int16_t M3508_SpeedLoopPIDController(pid_type_def *pid, fp32 ref, fp32 set)
  * @brief          3508速度环pid控制函数
  * @param[in]      pid:3508电机速度环pid句柄
  * @param[in]      ref:3508电机实际转速
  * @param[in]      set:3508电机目标转速
  * @retval         3508速度环pid输出值
  */
int16_t M3508_SpeedLoopPIDController(pid_type_def *pid, fp32 ref, fp32 set)
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
    else if (pid->mode == PID_DELTA)
    {
        pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
        pid->Iout = pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        pid->out += pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
		
		return pid->out;
    }
	else
	{
		return 0;
	}
}

/**
  * @func			int16_t M3508_CurrentLoopPIDController(pid_type_def *pid, fp32 ref, fp32 set)
  * @brief          3508电流环pid控制函数
  * @param[in]      pid:3508电机电流环pid句柄
  * @param[in]      ref:3508电机实际电流
  * @param[in]      set:3508电机目标电流
  * @retval         3508电流环pid输出值
  */
int16_t M3508_CurrentLoopPIDController(pid_type_def *pid, fp32 ref, fp32 set)
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
	else if (pid->mode == PID_DELTA)
    {
        pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
        pid->Iout = pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        pid->out += pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
		
		return pid->out;
    }
	else
	{
		return 0;
	}
}

