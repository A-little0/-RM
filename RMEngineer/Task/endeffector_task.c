#include "endeffector_task.h"


MOTOR_send yaw_motor_send;
MOTOR_recv yaw_motor_rev;

MOTOR_send pitch_motor_send;
MOTOR_recv pitch_motor_rev;

/*电磁阀控制 控制导管通气或阻塞*/
void SolenoidValve_Control(uint8_t status)
{
	//PE 13
	switch(status)
	{
		case SOLENOID_VALVE_OPEN:
			HAL_GPIO_WritePin(soleniod_valve_GPIO_Port,soleniod_valve_Pin,GPIO_PIN_SET);
			break;
		case SOLENOID_VALVE_CLOSE:
			HAL_GPIO_WritePin(soleniod_valve_GPIO_Port,soleniod_valve_Pin,GPIO_PIN_RESET);
			break;
		default:
			break;
	}
}
/*继电器控制 控制气泵电源导通或者关闭*/
void AirPump_Control(uint8_t status)
{
	//PE 14
	//PC 6
	switch(status)
	{
		case AIR_PUMP_POWER_OPEN:
			HAL_GPIO_WritePin(Relay_IN1_GPIO_Port,Relay_IN1_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(Relay_IN2_GPIO_Port,Relay_IN2_Pin,GPIO_PIN_SET);
			break;
		case AIR_PUMP_POWER_CLOSE:
			HAL_GPIO_WritePin(Relay_IN1_GPIO_Port,Relay_IN1_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(Relay_IN2_GPIO_Port,Relay_IN2_Pin,GPIO_PIN_RESET);
			break;
		default :
			break;
	}
}
void Endeffector_Task(float yaw_pos_angle,float pitch_pos_angle,float roll_dangle)
{
	yaw_motor_send.id=0; 			//给电机控制指令结构体赋值
	yaw_motor_send.mode=1;
	yaw_motor_send.T=0;
	yaw_motor_send.W=yaw_pos_angle;
	yaw_motor_send.Pos=0;
	yaw_motor_send.K_P=0;
	yaw_motor_send.K_W=0.05;
	SERVO_Send_recv(&yaw_motor_send, &yaw_motor_rev);
	yaw_motor_send.id=1; 			//给电机控制指令结构体赋值
	yaw_motor_send.mode=1;
	yaw_motor_send.T=0;
	yaw_motor_send.W=pitch_pos_angle;
	yaw_motor_send.Pos=0;
	yaw_motor_send.K_P=0;
	yaw_motor_send.K_W=0.05;
	SERVO_Send_recv(&pitch_motor_send, &pitch_motor_rev);
	AngleCloseControlLoop5(1,roll_dangle);
}