#include "detection_task.h"

int32_t rc_link_time;
int32_t rc_link_lasttime;


void Detection_TaskInit(void)
{
	rc_link_time=0;
	rc_link_lasttime=0;
}


int16_t Detection_ChassisPower(int16_t currentMA,int16_t currentMB,int16_t currentMC,int16_t currentMD)
{
	currentMA=currentMA >0 ? currentMA : -currentMA;
	currentMB=currentMB >0 ? currentMB : -currentMB;
	currentMC=currentMC >0 ? currentMC : -currentMC;
	currentMD=currentMD >0 ? currentMD : -currentMD;
	
	return (currentMA+currentMB+currentMC+currentMD)*24;	
}

int16_t Detection_GimbalPower(int16_t yaw_current,int16_t pitch_current)
{
	yaw_current = yaw_current > 0 ? yaw_current : -yaw_current;
	pitch_current =pitch_current > 0 ? pitch_current : -pitch_current;
	
	return (yaw_current + pitch_current)*24;
}

//���书�ʵĲ���������Ħ����(Ħ����Ϊsnail������ܷ���ʵ�ʵ���)
int16_t Detection_ShootPower(int16_t down_current,int16_t up_current)
{
	down_current = down_current > 0 ? down_current:-down_current;
	up_current =up_current > 0 ? up_current : -up_current;

	return (down_current+up_current)*24;
}

void Detection_RobotPowerTask(void)
{
	//���ʼ��Ӧ�ð���ƽ�����ʺ�˲ʱ������������
}