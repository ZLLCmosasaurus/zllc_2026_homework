/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ax_ps2.h"
#include "motor_control.h"
#include "servo_control.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
JOYSTICK_TypeDef ps2_data;  // PS2手柄数据

int16_t left_motor_target = 0;
int16_t right_motor_target = 0;
int16_t left_pwm_filtered = 0;
int16_t right_pwm_filtered = 0;

// 控制模式：0=PS2手柄模式, 1=上位机控制模式
uint8_t control_mode = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART3_UART_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
	// 初始化PS2手柄
	AX_PS2_Init();
	
	// 初始化电机
	Motor_Init();
	
	// 初始化舵机
	Servo_Init();
	
	// 启动串口中断接收
	UART_IT_Enable();
	
	// 等待一段时间确保所有外设完全初始化
	HAL_Delay(100);
	
	// 再次确保电机停止
	Car_Stop();
	
	// 额外延时确保系统稳定
	HAL_Delay(50);
	
	// 欢迎信息
	printf("\r\n=================================\r\n");
	printf("  PS2 Hybrid Car + Servo Control  \r\n");
	printf("=================================\r\n");
	printf("Mode: %s\r\n", control_mode == 0 ? "PS2" : "UART");
	printf("Motor: 4x Mecanum Wheels\r\n");
	printf("Servo: 4x MG996R\r\n");
	printf("Ready!\r\n\r\n");
	
	// 初始化PS2数据结构
	ps2_data.mode = 0;
	ps2_data.btn1 = 0;
	ps2_data.btn2 = 0;
	ps2_data.LJoy_LR = 127;
	ps2_data.LJoy_UD = 127;
	ps2_data.RJoy_LR = 127;
	ps2_data.RJoy_UD = 127;
	
	// 确保电机初始状态为停止
	Car_Stop();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	uint32_t last_print_time = 0;
	uint32_t last_control_time = 0;
	uint32_t last_speed_update_time = 0;   // 速度更新计时器
	uint32_t last_odometer_print_time = 0; // 里程显示计时器
	
  while (1)
  {
		uint32_t current_time = HAL_GetTick();
		
		// 每100ms更新一次速度测量
		if(current_time - last_speed_update_time >= 100)
		{
			last_speed_update_time = current_time;
			Speed_Update();
		}
		
		// 每1000ms打印一次里程和速度信息
		if(current_time - last_odometer_print_time >= 1000)
		{
			last_odometer_print_time = current_time;
			
			float left_dist = Odometer_GetLeftDistance();
			float right_dist = Odometer_GetRightDistance();
			float total_dist = Odometer_GetTotalDistance();
			float left_speed = Speed_GetLeft();
			float right_speed = Speed_GetRight();
			
			printf("[Odometer] L:%.1fmm R:%.1fmm Total:%.1fmm | Speed L:%.1fmm/s R:%.1fmm/s\r\n",
				   left_dist, right_dist, total_dist, left_speed, right_speed);
		}
		
		// 根据控制模式执行不同的控制逻辑
		// PS2 MODE键边沿检测与防抖（两种模式下都执行）
		static uint8_t last_ps2_mode_byte = 0;
		static uint32_t last_mode_toggle_time = 0;
		static uint32_t last_mode_scan_time = 0;
		if(current_time - last_mode_scan_time >= 20)
		{
			last_mode_scan_time = current_time;
			AX_PS2_ScanKey(&ps2_data);
			uint8_t controller_ok = (ps2_data.mode == 0x73 || ps2_data.mode == 0x41);
			if(controller_ok)
			{
				if(last_ps2_mode_byte == 0)
				{
					last_ps2_mode_byte = ps2_data.mode;
				}
				else if(ps2_data.mode != last_ps2_mode_byte && (current_time - last_mode_toggle_time) >= 300)
				{
					last_ps2_mode_byte = ps2_data.mode;
					last_mode_toggle_time = current_time;
					control_mode ^= 1; // 0<->1 切换
					printf("OK: Mode toggled to %d (%s) by PS2 MODE key\r\n", control_mode, control_mode == 0 ? "PS2" : "UART");
				}
			}
		}
		if(control_mode == 0)  // PS2手柄模式
		{
			// 每20ms读取一次PS2手柄数据
			if(current_time - last_control_time >= 20)
			{
				last_control_time = current_time;
				
				// 读取PS2手柄数据
				AX_PS2_ScanKey(&ps2_data);
				
				// 检查手柄是否连接正常
				if(ps2_data.mode == 0x73 || ps2_data.mode == 0x41)
				{
					// 使用左摇杆和按键控制小车
					// btn1: Bit1=JOYR, Bit2=JOYL, Bit4=UP, Bit5=RIGHT, Bit6=DOWN, Bit7=LEFT
					PS2_Control_Car(ps2_data.LJoy_LR, ps2_data.LJoy_UD, ps2_data.btn1);
					
					// 使用上下左右键和图形键控制舵机
					// btn1: 方向键, btn2: 图形键(△○×□)
					PS2_Control_Servo(ps2_data.btn1, ps2_data.btn2);
					
					// 每500ms打印一次手柄数据（可选）
					if(current_time - last_print_time >= 500)
					{
						last_print_time = current_time;
						printf("[PS2] LJ(%3d,%3d) RJ(%3d,%3d) Mode:0x%02X Btn1:0x%02X Btn2:0x%02X\r\n",
							   ps2_data.LJoy_LR, ps2_data.LJoy_UD, 
							   ps2_data.RJoy_LR, ps2_data.RJoy_UD,
							   ps2_data.mode, ps2_data.btn1, ps2_data.btn2);
					}
				}
				else
				{
					// 手柄未连接，停止小车
					Car_Stop();
					
					// 重置PS2数据以防止误触发
					ps2_data.btn1 = 0;
					ps2_data.btn2 = 0;
					ps2_data.LJoy_LR = 127;
					ps2_data.LJoy_UD = 127;
					ps2_data.RJoy_LR = 127;
					ps2_data.RJoy_UD = 127;
					
					if(current_time - last_print_time >= 2000)
					{
						last_print_time = current_time;
						printf("[PS2] No controller detected (Mode:0x%02X)\r\n", ps2_data.mode);
					}
				}
			}
		}
		// 如果是上位机模式，则由串口命令控制，这里不需要做什么
		
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
