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
#include "can.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include <stdio.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
cascade_pid_controller_t motor_cascade_pid;  // 串级PID控制器实例（现在仅用速度PID）
pid_controller_t speed_pid;  // 专用速度PID控制器
uint32_t last_pid_time = 0;  // 上次PID计算时间
uint8_t uart_tx_buffer[100];  // UART发送缓冲区
uint32_t last_heartbeat_time = 0;
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
  MX_USART1_UART_Init();  // 先初始化 UART，确保 CAN 接收回调可用
  MX_CAN_Init();          // 再初始化 CAN 并激活接收中断
  /* USER CODE BEGIN 2 */
  
  // 初始化串级PID控制器（现仅作为结构体容器）
  cascade_pid_init(&motor_cascade_pid, 0.002f, 0.01f, 0.1f, 0.002f, 0.01f, 0.1f);
  
  // 初始化速度PID控制器，参数可根据电机特性调整
  // Kp: 比例系数，控制响应速度
  // Ki: 积分系数，消除稳态误差
  // Kd: 微分系数，阻尼和预测
  // 增大Kp以提高响应，增大Ki以克服稳态误差
  pid_init(&speed_pid, 0.9f, 0.2f, 0.02f);
  
  // 设置目标速度为 0（初始不转，可在运行时修改）
  motor_cascade_pid.target_speed = 1000.0f;  // 目标速度，单位为电流命令值 (-16384 ~ 16384)
  motor_cascade_pid.target_angle = 0.0f;  // 角度已忽略

  // 初始化心跳时间基准
  last_heartbeat_time = HAL_GetTick();

  // 电机控制应在主循环中根据反馈数据进行
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // 获取当前系统时间
    uint32_t current_time = HAL_GetTick();
    
    // 每10ms执行一次速度PID控制 (100Hz)
    if(current_time - last_pid_time >= 10)
    {
      // 关键保护：仅当电机反馈已初始化时才执行控制
      // 否则发送 0 电流，避免在未收到反馈时对电机发送垃圾命令
      if (motor_angle_inited[0]) {
        // 计算时间间隔(秒)，最小防护防止极小时间间隔
        float dt = (current_time - last_pid_time) / 1000.0f;
        if (dt < 0.001f) dt = 0.001f;
        
        // 获取实际电机速度反馈
        float actual_speed = (float)motor_feedback[0].speed;
        
        // 调用速度PID计算器，基于目标速度和实际速度计算输出电流
        float output_current = pid_calculate(&speed_pid, motor_cascade_pid.target_speed, actual_speed, dt);
        
        // 限制输出在电机允许的范围内
        if (output_current > 16384.0f) output_current = 16384.0f;
        if (output_current < -16384.0f) output_current = -16384.0f;
        
        // 调试输出：发送易于波形工具解析的格式（逗号分隔）
        char debug_buffer[100];
        int len = snprintf(debug_buffer, sizeof(debug_buffer), "%d,%d,%d,%d\r\n",
                           (int)motor_cascade_pid.target_speed, 
                           motor_feedback[0].speed, 
                           (int)speed_pid.error, 
                           (int)output_current);
        HAL_UART_Transmit(&huart1, (uint8_t*)debug_buffer, len, 50);
        
        // 控制电机 (ID为1的电机)
        can_dji_motor_control(1, (int16_t)output_current);
      } else {
        // 电机反馈未初始化，发送 0 电流保持电机空闲
        can_dji_motor_control(1, 0);
      }
      
      // 更新上次执行时间
      last_pid_time = current_time;
    }
    
    // 添加一个小延迟以减少CPU占用
    HAL_Delay(1);

    // 每10ms发送一次完整状态（易于波形显示）
    if (HAL_GetTick() - last_heartbeat_time >= 50) {
      // 同时发送当前电机状态（CSV格式）
      if (motor_angle_inited[0]) {
        char status_buf[100];
        int slen = snprintf(status_buf, sizeof(status_buf), 
                           "%d,%d,%d\r\n",
                           (int)motor_cascade_pid.target_speed,
                           motor_feedback[0].speed,
                           (int)speed_pid.output);
        if (slen > 0 && slen < (int)sizeof(status_buf)) {
          HAL_UART_Transmit(&huart1, (uint8_t*)status_buf, (uint16_t)slen, 50);
        }
      }
      
      // 切换 PA0 以便示波器或板上指示灯可确认心跳
      HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_0);
      last_heartbeat_time = HAL_GetTick();
    }
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
