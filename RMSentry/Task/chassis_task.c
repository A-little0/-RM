
/**  
  ****************************(C) COPYRIGHT 2024 征途****************************
  * @file       chassis_task.c/h
  * @brief      
  *             这里是麦轮底盘任务程序，包含麦轮解算，底盘小陀螺，底盘跟随云台功能程序
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
#include "chassis_task.h"

/*底盘电机句柄声明*/
M3508_HandleTypeDef m3508_id1;//left front motor
M3508_HandleTypeDef m3508_id2;//right front motor
M3508_HandleTypeDef m3508_id3;//right rear motor
M3508_HandleTypeDef m3508_id4;//left rear motor
/*底盘坐标系*/
Chassis_CoordinateSystem_Typedef  chassis_vectors;
/*底盘相关pid*/
pid_type_def chassis_angle_pid;

/**
  * @func			void Chassis_Init(void)
  * @brief          底盘驱动初始化
  * @param[in]      none
  * @retval         none
  */
void Chassis_Init(void)
{
	/*底盘电机初始化*/
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
	/*底盘坐标系初始化*/
	chassis_vectors.c_x=0;
	chassis_vectors.c_y=0;
	chassis_vectors.c_rotate=0;
	chassis_vectors.cos=0.0f;
	chassis_vectors.sin=0.0f;
	
	chassis_vectors.beganangle_cwtw=0;//获取开机角度
	chassis_vectors.angle_cmtw=0;
	
	/*底盘相关pid初始化*/
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
  * @brief          麦轮底盘运动学解算
  * @brief          底盘坐标系(俯视图)：
	/\底盘正方向			/\x(roll)
	||					||
	||					||
	||					||
		y(pitch)《========
		 
  * @param[in]      chassisXvector:底盘坐标系下，x方向下的速度
  * @param[in]      chassisYvectory:底盘坐标系下，y方向下的速度
  * @param[in]      chassisRotatevector:底盘(世界)坐标系下，底盘旋转的速度
  * @param[in]      rotateK:旋转系数(与底盘长款轴距有关k=a+b)
  * @retval         none
  */
void Chassis_WheatWheel_Solution(int16_t chassisXvector,int16_t chassisYvectory,int16_t chassisRotatevector,int rotateK)
{
	/*获取底盘电机基础数据*/
	M3508_GetBasicData(&m3508_id1);
	M3508_GetBasicData(&m3508_id2);
	M3508_GetBasicData(&m3508_id3);
	M3508_GetBasicData(&m3508_id4);

	/*麦轮解算*/
	int16_t targetspeedMA=chassisXvector+chassisYvectory+chassisRotatevector*rotateK;
	int16_t targetspeedMD=chassisXvector-chassisYvectory+chassisRotatevector*rotateK;
	int16_t targetspeedMB=chassisXvector-chassisYvectory-chassisRotatevector*rotateK;
	int16_t targetspeedMC=chassisXvector+chassisYvectory-chassisRotatevector*rotateK;
	#if (USING_SMOOTHING_FILTER == 1)
	/*创建平滑滤波器*/
	static uint8_t getlengthMA=0;
	static int16_t filter_boxsMA[M3508_ID1_DATALENGTH];
	static uint8_t getlengthMB=0;
	static int16_t filter_boxsMB[M3508_ID2_DATALENGTH];
	static uint8_t getlengthMC=0;
	static int16_t filter_boxsMC[M3508_ID3_DATALENGTH];
	static uint8_t getlengthMD=0;
	static int16_t filter_boxsMD[M3508_ID4_DATALENGTH];
	/*平滑滤波*/
	int16_t filter_rpmMA=Smoothing_filter(m3508_id1.basic_data.speed_rpm,&getlengthMA,filter_boxsMA,M3508_ID1_DATALENGTH);
	int16_t filter_rpmMB=Smoothing_filter(m3508_id2.basic_data.speed_rpm,&getlengthMB,filter_boxsMB,M3508_ID2_DATALENGTH);
	int16_t filter_rpmMC=Smoothing_filter(m3508_id3.basic_data.speed_rpm,&getlengthMC,filter_boxsMC,M3508_ID3_DATALENGTH);
	int16_t filter_rpmMD=Smoothing_filter(m3508_id4.basic_data.speed_rpm,&getlengthMD,filter_boxsMD,M3508_ID4_DATALENGTH);
	/*pid运算*/
	int16_t set_currentMA=M3508_SpeedLoopPIDController(&(m3508_id1.speed_control_data), filter_rpmMA, targetspeedMA);
	int16_t set_currentMB=M3508_SpeedLoopPIDController(&(m3508_id2.speed_control_data), filter_rpmMB, -targetspeedMB);
	int16_t set_currentMC=M3508_SpeedLoopPIDController(&(m3508_id3.speed_control_data), filter_rpmMC, -targetspeedMC);
	int16_t set_currentMD=M3508_SpeedLoopPIDController(&(m3508_id4.speed_control_data), filter_rpmMD, targetspeedMD);	
	#else
	/*pid运算*/
	int16_t set_currentMA=M3508_SpeedLoopPIDController(&(m3508_id1.speed_control_data), m3508_id1.basic_data.speed_rpm, targetspeedMA);
	int16_t set_currentMB=M3508_SpeedLoopPIDController(&(m3508_id2.speed_control_data), m3508_id2.basic_data.speed_rpm, -targetspeedMB);
	int16_t set_currentMC=M3508_SpeedLoopPIDController(&(m3508_id3.speed_control_data), m3508_id3.basic_data.speed_rpm, -targetspeedMC);
	int16_t set_currentMD=M3508_SpeedLoopPIDController(&(m3508_id4.speed_control_data), m3508_id4.basic_data.speed_rpm, targetspeedMD);
	#endif
	/*发送电流至底盘电机*/
	CAN_cmd_chassis(set_currentMA,set_currentMB,set_currentMC,set_currentMD);
	//CAN_cmd_chassis(set_currentMA,0,0,0);
}

/**
  * @func			void ChassisGyro_Task(
  *						int16_t worldXvector,
  *						int16_t worldYvector,
  *						int rotateK,
  *						float c_angle)
  * @brief          底盘小陀螺任务
  * @param[in]      worldXvector：世界/云台坐标系下，x方向速度
  * @param[in]      worldYvector：世界/云台坐标系下，y方向速度
  * @param[in]      rotateK：旋转系数(与底盘长款轴距有关k=a+b)
  * @param[in]      c_angle：世界/云台坐标系与底盘坐标系的夹角
  * @retval         none
  */
void ChassisGyro_Task(int16_t worldXvector,int16_t worldYvector,int rotateK,float c_angle)
{
	/*绝对编码器值转为角度*/
	chassis_vectors.angle_cmtw=c_angle*360/8191;
	/*cos,sin计算*/
	arm_sin_cos_f32(chassis_vectors.angle_cmtw,&chassis_vectors.sin,&chassis_vectors.cos);
	/*将世界坐标系下的目标速度映射到底盘坐标系*/
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
  * @brief          底盘跟随云台任务
  * @param[in]      worldXvector：世界/云台坐标系下，x方向速度
  * @param[in]      worldYvector：世界/云台坐标系下，y方向速度
  * @param[in]      rotateK：旋转系数(与底盘长款轴距有关k=a+b)
  * @param[in]      realangle：  世界/云台坐标系与底盘坐标系的实时夹角(yaw轴电机绝对编码器值)
  * @param[in]      targetangle：世界/云台坐标系与底盘坐标系的目标夹角(yaw轴电机绝对编码器值)
  * @retval         none
  */
void ChassisPositionControl_Task(int16_t worldXvector,int16_t worldYvector,int rotateK,float32_t realangle,float32_t targetangle)
{
	
	/*串级pid*/
	int16_t targetspeed=PID_calc(&chassis_angle_pid, realangle, targetangle);//角度环
	/*绝对不是因为懒才不用角加速度环*/
	//int16_t output=PID_calc(&chassis_whellspeed_pid, chassis_angul, targetspeed);//角加速速度环
	
	Chassis_WheatWheel_Solution(worldXvector,worldYvector,targetspeed,rotateK);//速度环
}

/**
  * @func			void Printf_ChassisMessage(int16_t targetspeed)
  * @brief          底盘调试任务
  * @param[in]      targetangle：各底盘电机目标速度
  * @retval         none
  */
void Printf_ChassisMessage(int16_t targetspeed)
{
	printf("%d,%d,%d,%d,%d\n",m3508_id1.basic_data.speed_rpm,-m3508_id2.basic_data.speed_rpm,-m3508_id3.basic_data.speed_rpm,m3508_id4.basic_data.speed_rpm,targetspeed);
}
