/**
  ****************************(C) COPYRIGHT 2024 ��;****************************
  * @file       3508_driver.c/h
  * @brief      
  *             ������3508������������ͨ��pid���ƿ��Ƶ��.
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
#include "3508_driver.h"

long int exp_ecd=0;

/**
  * @func			void M3508_Init(uint8_t motorid,M3508_HandleTypeDef* device,enum PID_MODE mode)
  * @brief          3508�����ز�����ʼ��
  * @param[in]      motorid:��ʼ�����id
  * @param[in]      device:3508������
  * @param[in]      pidmode:pid�Ŀ���ģʽ(����ʽ��λ��ʽ)
  * @retval         none
  */
void M3508_Init(uint8_t motorid,M3508_HandleTypeDef* device,enum PID_MODE mode)
{
	//��ʼ�����id
	device->motor_id=motorid;
	//���������Ϣ��ʼ��
	device->basic_data.ecd=0;
	device->basic_data.given_current=0;
	device->basic_data.last_ecd=0;
	device->basic_data.speed_rpm=0;
	device->basic_data.temperate=0;
	//
	device->expand_data.exp_cycles=0;
	device->expand_data.exp_ecd=0;
	//��ʼ��pid
	fp32 PID_SPEED[3]={0};//�ٶȻ�pid����
	fp32 PID_CURRENT[3]={0};//������pid����
	fp32 PID_ECD[3]={0};
	
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
			case CAN_LIFITING_LEFT_MOTOR_ID:
		    PID_SPEED[0]=LIFITING_LEFT_SPEEDPID_POSITION_KP;
			PID_SPEED[1]=LIFITING_LEFT_SPEEDPID_POSITION_KI;
			PID_SPEED[2]=LIFITING_LEFT_SPEEDPID_POSITION_KD;
			
			PID_ECD[0]=LIFITING_LEFT_EXPECDPID_POSITION_KP;
			PID_ECD[1]=LIFITING_LEFT_EXPECDPID_POSITION_KI;
			PID_ECD[2]=LIFITING_LEFT_EXPECDPID_POSITION_KD;
				break;
			case CAN_LIFITING_RIGHT_MOTOR_ID:
			PID_SPEED[0]=LIFITING_RIGHT_SPEEDPID_POSITION_KP;
			PID_SPEED[1]=LIFITING_RIGHT_SPEEDPID_POSITION_KI;
			PID_SPEED[2]=LIFITING_RIGHT_SPEEDPID_POSITION_KD;
			
			PID_ECD[0]=LIFITING_RIGHT_EXPECDPID_POSITION_KP;
			PID_ECD[1]=LIFITING_RIGHT_EXPECDPID_POSITION_KI;
			PID_ECD[2]=LIFITING_RIGHT_EXPECDPID_POSITION_KD;
				break;
			case CAN_DRAWER_LEFT_MOTOR_ID:
		    PID_SPEED[0]=DRAWER_LEFT_SPEEDPID_POSITION_KP;
			PID_SPEED[1]=DRAWER_LEFT_SPEEDPID_POSITION_KI;
			PID_SPEED[2]=DRAWER_LEFT_SPEEDPID_POSITION_KD;
				break;
			case CAN_DRAWER_RIGHT_MOTOR_ID:
				break;
			default :
				break;
		}
	}
	PID_init(&(device->speed_control_data), mode, PID_SPEED, 16384,16384);
	PID_init(&(device->currrent_control_data), mode, PID_CURRENT, 16384,16384);
	PID_init(&(device->expecd_control_data), mode,PID_ECD, 4000,4000);
	
}

/**
  * @func			void M3508_GetBasicData(M3508_HandleTypeDef* device)
  * @brief          ��ȡ3508�����ز���
  * @param[in]      device:3508������
  * @retval         none
  */
void M3508_GetBasicData(M3508_HandleTypeDef* device)
{
	if(device==NULL)
	{
		return;
	}
	/*���id�ж�*/
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
		case CAN_LIFITING_LEFT_MOTOR_ID:
        case CAN_LIFITING_RIGHT_MOTOR_ID:
        case CAN_DRAWER_LEFT_MOTOR_ID:
		case CAN_DRAWER_RIGHT_MOTOR_ID:
		device->basic_data.ecd=get_gimbal_motor_measure_point(device->motor_id-1)->ecd;
		device->basic_data.given_current=get_gimbal_motor_measure_point(device->motor_id-1)->given_current;
		device->basic_data.last_ecd=get_gimbal_motor_measure_point(device->motor_id-1)->last_ecd;
		device->basic_data.speed_rpm=get_gimbal_motor_measure_point(device->motor_id-1)->speed_rpm;
		device->basic_data.temperate=get_gimbal_motor_measure_point(device->motor_id-1)->temperate;			
		break;
		default:
			break;
			
	}
}


void M3508_ExpandECD(M3508_HandleTypeDef* device)
{
	static int16_t now_ecd=0;
	static int16_t last_ecd=0;
	static  int now_cycles=0;
	//long int exp_ecd=0;

	now_ecd=device->basic_data.ecd;
	int16_t d_ecd=now_ecd-last_ecd;
	
	now_cycles=device->expand_data.exp_cycles;
	/*�жϾ��Ա�����ֵ�Ƿ���ͻ��*/
	if(d_ecd <= -8000)
	{
		/*����ͻ��Ϊ����ͻ��*/
		device->expand_data.exp_cycles++;
	}
	else if(d_ecd >=8000)
	{
		/*����ͻ��Ϊ����ͻ��*/
		device->expand_data.exp_cycles--;
	}

//	last_ecd=now_ecd;
//	//device->expand_data.exp_cycles*8191+device->basic_data.ecd
//	//device->expand_data.exp_ecd=now_cycles+now_ecd;
//	exp_ecd=now_cycles*8191+now_ecd;
//	//device->expand_data.exp_ecd=exp_ecd;
}
/**
  * @func			int16_t M3508_SpeedLoopPIDController(pid_type_def *pid, fp32 ref, fp32 set)
  * @brief          3508�ٶȻ�pid���ƺ���
  * @param[in]      pid:3508����ٶȻ�pid���
  * @param[in]      ref:3508���ʵ��ת��
  * @param[in]      set:3508���Ŀ��ת��
  * @retval         3508�ٶȻ�pid���ֵ
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
	else
	{
		return 0;
	}
}

/**
  * @func			int16_t M3508_CurrentLoopPIDController(pid_type_def *pid, fp32 ref, fp32 set)
  * @brief          3508������pid���ƺ���
  * @param[in]      pid:3508���������pid���
  * @param[in]      ref:3508���ʵ�ʵ���
  * @param[in]      set:3508���Ŀ�����
  * @retval         3508������pid���ֵ
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
	else
	{
		return 0;
	}
}

int16_t M3508_SpeedSyncPIDController(pid_type_def* pid ,fp32 ref,fp32 set)
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


int16_t M3508_ExpandEcdPIDController(pid_type_def* pid ,fp32 ref,fp32 set)
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
