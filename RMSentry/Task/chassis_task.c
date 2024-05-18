
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
#include "chassis_task.h"

/*���̵���������*/
M3508_HandleTypeDef m3508_id1;//left front motor
M3508_HandleTypeDef m3508_id2;//right front motor
M3508_HandleTypeDef m3508_id3;//right rear motor
M3508_HandleTypeDef m3508_id4;//left rear motor
/*��������ϵ*/
Chassis_CoordinateSystem_Typedef  chassis_vectors;
/*�������pid*/
pid_type_def chassis_angle_pid;

/**
  * @func			void Chassis_Init(void)
  * @brief          ����������ʼ��
  * @param[in]      none
  * @retval         none
  */
void Chassis_Init(void)
{
	/*���̵����ʼ��*/
	#if (CHASSiS_SPEED_PID_MODE == 0)
	M3508_Init(1,&m3508_id1,PID_POSITION);
	M3508_Init(2,&m3508_id2,PID_POSITION);
	M3508_Init(3,&m3508_id3,PID_POSITION);
	M3508_Init(4,&m3508_id4,PID_POSITION);
	#else
	M3508_Init(1,&m3508_id1,PID_DELTA);
	M3508_Init(2,&m3508_id2,PID_DELTA);
	M3508_Init(3,&m3508_id3,PID_DELTA);
	M3508_Init(4,&m3508_id4,PID_DELTA);
	#endif
	/*��������ϵ��ʼ��*/
	chassis_vectors.c_x=0;
	chassis_vectors.c_y=0;
	chassis_vectors.c_rotate=0;
	chassis_vectors.cos=0.0f;
	chassis_vectors.sin=0.0f;
	
	chassis_vectors.beganangle_cwtw=0;//��ȡ�����Ƕ�
	chassis_vectors.angle_cmtw=0;
	
	/*�������pid��ʼ��*/
	fp32 pid_angle[3]={
	 CHASSIS_ANGLEPID_KP,
	 CHASSIS_ANGLEPID_KI,
     CHASSIS_ANGLEPID_KD
	};
	PID_init(&chassis_angle_pid,PID_POSITION,pid_angle,  5000,5000);
}

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
void Chassis_WheatWheel_Solution(int16_t chassisXvector,int16_t chassisYvectory,int16_t chassisRotatevector,int rotateK)
{
	/*��ȡ���̵����������*/
	M3508_GetBasicData(&m3508_id1);
	M3508_GetBasicData(&m3508_id2);
	M3508_GetBasicData(&m3508_id3);
	M3508_GetBasicData(&m3508_id4);

	/*���ֽ���*/
	int16_t targetspeedMA=chassisXvector+chassisYvectory+chassisRotatevector*rotateK;
	int16_t targetspeedMD=chassisXvector-chassisYvectory+chassisRotatevector*rotateK;
	int16_t targetspeedMB=chassisXvector-chassisYvectory-chassisRotatevector*rotateK;
	int16_t targetspeedMC=chassisXvector+chassisYvectory-chassisRotatevector*rotateK;
	#if (USING_SMOOTHING_FILTER == 1)
	/*����ƽ���˲���*/
	static uint8_t getlengthMA=0;
	static int16_t filter_boxsMA[M3508_ID1_DATALENGTH];
	static uint8_t getlengthMB=0;
	static int16_t filter_boxsMB[M3508_ID2_DATALENGTH];
	static uint8_t getlengthMC=0;
	static int16_t filter_boxsMC[M3508_ID3_DATALENGTH];
	static uint8_t getlengthMD=0;
	static int16_t filter_boxsMD[M3508_ID4_DATALENGTH];
	/*ƽ���˲�*/
	int16_t filter_rpmMA=Smoothing_filter(m3508_id1.basic_data.speed_rpm,&getlengthMA,filter_boxsMA,M3508_ID1_DATALENGTH);
	int16_t filter_rpmMB=Smoothing_filter(m3508_id2.basic_data.speed_rpm,&getlengthMB,filter_boxsMB,M3508_ID2_DATALENGTH);
	int16_t filter_rpmMC=Smoothing_filter(m3508_id3.basic_data.speed_rpm,&getlengthMC,filter_boxsMC,M3508_ID3_DATALENGTH);
	int16_t filter_rpmMD=Smoothing_filter(m3508_id4.basic_data.speed_rpm,&getlengthMD,filter_boxsMD,M3508_ID4_DATALENGTH);
	/*pid����*/
	int16_t set_currentMA=M3508_SpeedLoopPIDController(&(m3508_id1.speed_control_data), filter_rpmMA, targetspeedMA);
	int16_t set_currentMB=M3508_SpeedLoopPIDController(&(m3508_id2.speed_control_data), filter_rpmMB, -targetspeedMB);
	int16_t set_currentMC=M3508_SpeedLoopPIDController(&(m3508_id3.speed_control_data), filter_rpmMC, -targetspeedMC);
	int16_t set_currentMD=M3508_SpeedLoopPIDController(&(m3508_id4.speed_control_data), filter_rpmMD, targetspeedMD);	
	#else
	/*pid����*/
	int16_t set_currentMA=M3508_SpeedLoopPIDController(&(m3508_id1.speed_control_data), m3508_id1.basic_data.speed_rpm, targetspeedMA);
	int16_t set_currentMB=M3508_SpeedLoopPIDController(&(m3508_id2.speed_control_data), m3508_id2.basic_data.speed_rpm, -targetspeedMB);
	int16_t set_currentMC=M3508_SpeedLoopPIDController(&(m3508_id3.speed_control_data), m3508_id3.basic_data.speed_rpm, -targetspeedMC);
	int16_t set_currentMD=M3508_SpeedLoopPIDController(&(m3508_id4.speed_control_data), m3508_id4.basic_data.speed_rpm, targetspeedMD);
	#endif
	/*���͵��������̵��*/
	CAN_cmd_chassis(set_currentMA,set_currentMB,set_currentMC,set_currentMD);
	//CAN_cmd_chassis(set_currentMA,0,0,0);
}

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
void ChassisGyro_Task(int16_t worldXvector,int16_t worldYvector,int rotateK,float c_angle)
{
	/*���Ա�����ֵתΪ�Ƕ�*/
	chassis_vectors.angle_cmtw=c_angle*360/8191;
	/*cos,sin����*/
	arm_sin_cos_f32(chassis_vectors.angle_cmtw,&chassis_vectors.sin,&chassis_vectors.cos);
	/*����������ϵ�µ�Ŀ���ٶ�ӳ�䵽��������ϵ*/
	chassis_vectors.c_y=  worldXvector*chassis_vectors.sin+worldYvector*chassis_vectors.cos;
	chassis_vectors.c_x= -worldYvector*chassis_vectors.sin+worldXvector*chassis_vectors.cos;
	
	Chassis_WheatWheel_Solution(chassis_vectors.c_x,chassis_vectors.c_y,CHASSIS_GYRO_ROTATE_SPEED,1);
}

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
void ChassisPositionControl_Task(int16_t worldXvector,int16_t worldYvector,int rotateK,float32_t realangle,float32_t targetangle)
{
	
	/*����pid*/
	int16_t targetspeed=PID_calc(&chassis_angle_pid, realangle, targetangle);//�ǶȻ�
	/*���Բ�����Ϊ���Ų��ýǼ��ٶȻ�*/
	//int16_t output=PID_calc(&chassis_whellspeed_pid, chassis_angul, targetspeed);//�Ǽ����ٶȻ�
	
	Chassis_WheatWheel_Solution(worldXvector,worldYvector,targetspeed,rotateK);//�ٶȻ�
}

/**
  * @func			void Printf_ChassisMessage(int16_t targetspeed)
  * @brief          ���̵�������
  * @param[in]      targetangle�������̵��Ŀ���ٶ�
  * @retval         none
  */
void Printf_ChassisMessage(int16_t targetspeed)
{
	printf("%d,%d,%d,%d,%d\n",m3508_id1.basic_data.speed_rpm,-m3508_id2.basic_data.speed_rpm,-m3508_id3.basic_data.speed_rpm,m3508_id4.basic_data.speed_rpm,targetspeed);
}
