/**
  ****************************(C) COPYRIGHT 2024 ��;****************************
  * @file       6020_driver.c/h
  * @brief      
  *             ������6020������������ͨ��pid���ƿ��Ƶ��.
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
#include "6020_driver.h"

/**
  * @func			void M6020_Init(uint8_t motorid,M6020_HandleTypeDef* device,enum PID_MODE pidmode,enum CAN_CMD_MODE cmdmode)
  * @brief          6020�����ز�����ʼ��
  * @param[in]      motorid:��ʼ�����id
  * @param[in]      device:6020������
  * @param[in]      pidmode:pid�Ŀ���ģʽ(����ʽ��λ��ʽ)
  * @param[in]      cmdmode:6020�Ŀ���ģʽ( CAN_CMD_VOLTAGE��can���͵�ѹ�����źţ�CAN_CMD_CURRENT��can���͵��������ź�)
  * @retval         none
  */
void M6020_Init(uint8_t motorid,M6020_HandleTypeDef* device,enum PID_MODE pidmode,enum CAN_CMD_MODE cmdmode)
{
	if(device==NULL)
	{
		return;
	}
	/*��ʼ�������ز���*/
	device->motor_id=motorid;
	device->cmd_mode=cmdmode;
	
	device->basic_data.ecd=0;
	device->basic_data.given_current=0;
	device->basic_data.last_ecd=0;
	device->basic_data.speed_rpm=0;
	device->basic_data.temperate=0;
	/*��ʼ��pid��ز���*/
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
			/*��ʼ�����ַ��뷧��ֵ*/
			integralvalve=YAW_INTERGRAL_VALVE;
				break;
			case  CAN_PIT_MOTOR_ID:
				
			pid_position[0]=PITCHM6020_POSITIONPID_KP;
			pid_position[1]=PITCHM6020_POSITIONPID_KI;
			pid_position[2]=PITCHM6020_POSITIONPID_KD;
			
			pid_speed[0]=PITCHM6020_SPEEDPID_KP;
			pid_speed[1]=PITCHM6020_SPEEDPID_KI;
			pid_speed[2]=PITCHM6020_SPEEDPID_KD;
			/*��ʼ�����ַ��뷧��ֵ*/
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
  * @brief          ��ȡ6020�����ز���
  * @param[in]      device:6020������
  * @retval         none
  */
void M6020_GetBasicData(M6020_HandleTypeDef* device)
{
	if(device==NULL)
	{
		return;
	}
	/*���id�ж�*/
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
  * @brief          6020λ�û�pid���ƺ���
  * @param[in]      pid:6020���λ�û�pid���
  * @param[in]      ref:6020���ʵ�ʽǶ�
  * @param[in]      set:6020���Ŀ��Ƕ�
  * @retval         6020λ�û�pid���ֵ
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
		//���ַ���
		if(absoulte_value(pid->error[0])<pid->integral_valve)
		{
			pid->Iout += pid->Ki * pid->error[0];
		}
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
		//�����޷�
        LimitMax(pid->Iout, pid->max_iout);
        pid->out = pid->Pout + pid->Iout + pid->Dout;
		//����޷�
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
  * @brief          2006�ٶȻ�pid���ƺ���
  * @param[in]      pid:6020����ٶȻ�pid���
  * @param[in]      ref:6020���ʵ��ת��
  * @param[in]      set:6020���Ŀ��ת��
  * @retval         6020�ٶȻ�pid���ֵ
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
  * @brief          6020������pid���ƺ���
  * @param[in]      pid:6020���������pid���
  * @param[in]      ref:6020���ʵ�ʵ���
  * @param[in]      set:6020���Ŀ�����
  * @retval         6020������pid���ֵ
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


	
