#ifndef __ENDEFFECTOR_TASK_H
#define  __ENDEFFECTOR_TASK_H

#include "MG4005V2_driver.h"
#include "motor_control.h"

#define SOLENOID_VALVE_OPEN  1//开启电磁阀为通气状态,失能气泵
#define SOLENOID_VALVE_CLOSE 0//关闭电磁阀为阻塞状态,使能气泵
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

/*电磁阀控制 控制导管通气或阻塞*/
void SolenoidValve_Control(uint8_t status);
/*继电器控制 控制气泵电源导通或者关闭*/
void AirPump_Control(uint8_t status);
void Endeffector_Task(float yaw_pos_angle,float pitch_pos_angle,float roll_dangle);
#endif
