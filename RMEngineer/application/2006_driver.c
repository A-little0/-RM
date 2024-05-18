/**
  ****************************(C) COPYRIGHT 2024 ��;****************************
  * @file       2006_driver.c/h
  * @brief      
  *             ������2006������������ͨ��pid���ƿ��Ƶ��.
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
#include "2006_driver.h"

//M2006_HandleTypeDef up_trigger_motor;
//M2006_HandleTypeDef down_trigger_motor;

/**
  * @func			void M2006_Init(uint8_t motorid,M2006_HandleTypeDef* device,enum PID_MODE pidmode)
  * @brief          2006�����ز�����ʼ��
  * @param[in]      motorid:��ʼ�����id
  * @param[in]      device:2006������
  * @param[in]      pidmode:pid�Ŀ���ģʽ(����ʽ��λ��ʽ)
  * @retval         none
  */
void M2006_Init(uint8_t motorid,M2006_HandleTypeDef* device,enum PID_MODE pidmode)
{
	/*��ʼ�������ز���*/
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
	
	/*��ʼ��pid��ز���*/
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
  * @brief          ��ȡ2006�����ز���
  * @param[in]      device:2006������
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
  * @brief          ��ȡ2006������������ԽǶ�(�Ա������Ƕȷ�Χ����������)
  * @brief          �ú����Ե�����ת�ĵ����Ч��������ת���ܻ���ڶ�Ȧ����
  * @param[in]      device:2006������
  * @retval         �����ı������Ƕ�ֵ
  */
int16_t M2006_GetExpandECDData(M2006_HandleTypeDef* device)
{
	if(device==NULL)
	{
		return 0;
	}
	//����һ��λ��-��һ��λ���ж��Ƿ���ͻ��
	int16_t d_ecd=device->basic_data.ecd-device->record_last_ecd;
	/*��ͻ���¼һȦ*/
	if(d_ecd>8000)//��תͻ��
	{
		device->cylinder_number--;
	}
	else if(d_ecd<-8000)//��תͻ��
	{
		device->cylinder_number++;
	}
	else//δͻ��
	{
		/*���㷶Χ��չ��ľ��Ա�����ֵ*/ 
		if(device->cylinder_number>=0)
		{
			device->expand_ecd=device->cylinder_number*8191+device->basic_data.ecd;
		}
		else
		{
			device->expand_ecd=device->cylinder_number*8191+8191-device->basic_data.ecd+8191;
		}                
	}
	/*������һ�ξ��Ա�����ֵ*/
	device->record_last_ecd=device->basic_data.ecd;
	
	return device->expand_ecd;
}

/**
  * @func			float M2006_AbsoluteEncoderToAngle(int absoluteencoder)
  * @brief          ��2006���(ת��)���������ԽǶ�ֵ ת��Ϊ (������)���������ԽǶ�ֵ
  * @param[in]      absoluteencoder:2006���(ת��)���������ԽǶ�ֵ
  * @retval         (������)���������ԽǶ�ֵ
  */
float M2006_AbsoluteEncoderToAngle(int absoluteencoder)
{
	//ת�ӽǶ�ת��
	float rev_angle=(float)absoluteencoder*360.0f/8191;
	//������Ƕ�ת��
	rev_angle/=M2006_REDUCTION_RATIO;
	
	return rev_angle;
}

//Locked-rotor detection
/**
  * @func			_userbool M2006_LockedMotorDetertion(M2006_HandleTypeDef* device)
  * @brief          2006�����ת���
  * @param[in]      device:2006������
  * @retval         boolֵ��user_trueΪ�����ת������Ϊ���δ��ת
  */
_userbool M2006_LockedMotorDetertion(M2006_HandleTypeDef* device)
{
	if(device==NULL)
	{
		return 0;
	}
	/*�Զ�ת������Ŀ���ٶ�Ϊ�����ת�о�*/
	if(absoulte_value(device->basic_data.given_current)>9000 && device->speed_control_data.set!=0)
	{
		//Ŀ���ٶȲ�Ϊ�㣬�ҵ���ֵ�ܴ��ж�Ϊ�����ת
		return user_true;
	}
	else{
		return user_false;
	}
}

/**
  * @func			int16_t M2006_PositionLoopPIDController(pid_type_def *pid, fp32 ref, fp32 set)
  * @brief          2006λ�û�pid���ƺ���
  * @param[in]      pid:2006���λ�û�pid���
  * @param[in]      ref:2006���ʵ�ʽǶ�
  * @param[in]      set:2006���Ŀ��Ƕ�
  * @retval         2006λ�û�pid���ֵ
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
  * @brief          2006�ٶȻ�pid���ƺ���
  * @param[in]      pid:2006����ٶȻ�pid���
  * @param[in]      ref:2006���ʵ��ת��
  * @param[in]      set:2006���Ŀ��ת��
  * @retval         2006�ٶȻ�pid���ֵ
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
  * @brief          2006������pid���ƺ���
  * @param[in]      pid:2006���������pid���
  * @param[in]      ref:2006���ʵ�ʵ���
  * @param[in]      set:2006���Ŀ�����
  * @retval         2006������pid���ֵ
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
