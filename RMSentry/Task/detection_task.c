#include "detection_task.h"


int32_t rc_link_time;
int32_t rc_link_lasttime;
RobotPowerData_HandleTypeDef robo_powerdata;

void Detection_TaskInit(void)
{
	rc_link_time=0;
	rc_link_lasttime=0;
	
	robo_powerdata.chassis_instantpower=0;
	robo_powerdata.gimbal_instantpower=0;
	robo_powerdata.shoot_instantpower=0;
	
	robo_powerdata.chassis_averagepower=0;
	robo_powerdata.gimbal_averagepower=0;
	robo_powerdata.shoot_averagepower=0;
}


float Detection_ChassisPower(int16_t currentMA,int16_t currentMB,int16_t currentMC,int16_t currentMD)
{
	currentMA=currentMA >0 ? currentMA : -currentMA;
	currentMB=currentMB >0 ? currentMB : -currentMB;
	currentMC=currentMC >0 ? currentMC : -currentMC;
	currentMD=currentMD >0 ? currentMD : -currentMD;
	
	return (currentMA+currentMB+currentMC+currentMD)*24*4/100000;	
}

int Detection_GimbalPower(int16_t yaw_current,int16_t pitch_current)
{
	yaw_current = yaw_current > 0 ? yaw_current : -yaw_current;
	pitch_current =pitch_current > 0 ? pitch_current : -pitch_current;
	
	return (yaw_current + pitch_current)*24;
}

//发射功率的测量不包含摩擦轮(摩擦轮为snail电机不能反馈实际电流)
int Detection_ShootPower(int16_t down_current,int16_t up_current)
{
	down_current = down_current > 0 ? down_current:-down_current;
	up_current =up_current > 0 ? up_current : -up_current;

	return (down_current+up_current)*24;
}

void Detection_RobotPowerTask(void)
{
	//功率检测应该包含平均功率和瞬时功率两个部分
	robo_powerdata.chassis_instantpower=Detection_ChassisPower(
		m3508_id1.basic_data.given_current,
		m3508_id2.basic_data.given_current,
		m3508_id3.basic_data.given_current,
		m3508_id4.basic_data.given_current
	);
}
