#ifndef __ENDEFFECTOR_TASK_H
#define  __ENDEFFECTOR_TASK_H

#include "MG4005V2_driver.h"
#include "motor_control.h"

#define SOLENOID_VALVE_OPEN  1//������ŷ�Ϊͨ��״̬,ʧ������
#define SOLENOID_VALVE_CLOSE 0//�رյ�ŷ�Ϊ����״̬,ʹ������
#define AIR_PUMP_POWER_OPEN  1
#define AIR_PUMP_POWER_CLOSE 0

typedef enum{
	
	AIR_PUMP_POWER_ENABLE_MODE =1,
	SOLENOID_VALVE_ENABLE_MODE=2,
	AIR_PUMP_POWER_DISABLE_MODE=3
	
}AIR_PUMP_CONTROL_MODE ;

extern MOTOR_send yaw_motor_send;
extern MOTOR_recv yaw_motor_rev;

extern MOTOR_send pitch_motor_send;
extern MOTOR_recv pitch_motor_rev;

/*��ŷ����� ���Ƶ���ͨ��������*/
void SolenoidValve_Control(uint8_t status);
/*�̵������� �������õ�Դ��ͨ���߹ر�*/
void AirPump_Control(uint8_t status);
void Endeffector_Task(float yaw_pos_angle,float pitch_pos_angle,float roll_dangle);
#endif
