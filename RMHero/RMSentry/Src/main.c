/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp_can.h"
#include "CAN_receive.h"
#include "rc_receive.h"
#include "chassis_task.h"
#include "gimbal_task.h"
#include "shoot_task.h"
#include "rc_task.h"
#include "arm_math.h"
#include "jy901.h"
#include "detection_task.h"
#include "demo.h"

int tim_watch=0;
int filter_d_k=0;
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
int setpwm=1100;
int16_t d_targetspeed=0;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
struct FILE{int handle;};
int fputc(int ch,struct FILE* f)
{
	HAL_UART_Transmit(&huart1,(uint8_t*)&ch,1,0xffff);
	return ch;
}
float feedback_k=0;
int16_t feedback_output=0;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern int16_t target_position;
const int length =3;
int16_t rpm_boxs[length]={0};
int16_t yaw_targetpostion=4000;
int16_t pitch_targetpostion=2842;
int16_t trigger_targetspeed=0;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
	if(htim->Instance==TIM2)
	{
		//Chassis_WheatWheel_Solution(d_targetspeed,0,0,1);
		tim_watch++;
		Demo();
//		M3508_GetBasicData(&m3508_id2);
//		d_targetspeed=rc_user.x;
//		int16_t set_currentMA=M3508_SpeedLoopPIDController(&(m3508_id2.speed_control_data), m3508_id2.basic_data.speed_rpm,d_targetspeed);
//        CAN_cmd_chassis(0,set_currentMA,0,0);
//		printf("%d,%d\n",m3508_id2.basic_data.speed_rpm,d_targetspeed);
		//CAN_cmd_chassis(0,5000,0,0);   
		//Chassis_WheatWheel_Solution(d_targetspeed,0,0,1);
		//Printf_ChassisMessage(d_targetspeed);
//		Chassis_WheatWheel_Solution(rc_user.x,0,0,1);
//		Printf_ChassisMessage(rc_user.x);
		//M6020_GetBasicData(&pitch_gimbal_motor);
		//int16_t pitch_output=Pitch_Gimbal_Task(rc_user.pitch);
		//int16_t yaw_output=Yaw_Gimbal_Task(rc_user.yaw);
//		int16_t pitch_output=Pitch_Gimbal_Task(pitch_targetpostion);//rc_user.pitch
		//CAN_cmd_gimbal(pitch_output,yaw_output , 0, 0);
		//printf("%d,%d\n",yaw_gimbal_motor.basic_data.ecd,rc_user.yaw);
		//printf("%d,%d\n",pitch_gimbal_motor.basic_data.ecd,rc_user.pitch);
		//printf("%d\n",1);
//		int16_t trigger1_output=SetFrition_Shoot_Task(&left_frition_motor,trigger_targetspeed);
//		int16_t trigger2_output=SetFrition_Shoot_Task(&right_frition_motor,trigger_targetspeed);
//		CAN_cmd_gimbal(0,0 ,trigger1_output, trigger2_output);
//		printf("%d,%d\n",-right_frition_motor.basic_data.speed_rpm,+trigger_targetspeed);
//		M3508_GetBasicData(&trigger_motor);
//		int16_t trigger3_output=SetTrigger_Shoot_Task(&trigger_motor,trigger_targetspeed);
//		CAN_cmd_shoot(trigger3_output,0,0,0);
//		printf("%d,%d\n",trigger_motor.basic_data.speed_rpm,trigger_targetspeed);
	 }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Init-+ializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
  JY901_Init();
  can_filter_init();
  HAL_TIM_Base_Start_IT(&htim2);//任务定时器
  RC_Receive_Init(); 
  Chassis_Init();
  Gimbal_Init();
  Shoot_Init();
  RC_TaskInit();
  Detection_TaskInit();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
 while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle) 
{  
	if(UartHandle->Instance==USART3)
	{
		//RC_Receive();
	}
	if(UartHandle->Instance==USART6)
	{
//		JY901_Process(&JY901_data);
//		JY901_ExpandAngle(&JY901_data);
	}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
