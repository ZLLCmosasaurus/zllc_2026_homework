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
#include <math.h>
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
cascade_pid_controller_t motor_cascade_pid;
float desired_angle = 0.0f;         // 期望目标角度
float desired_speed = 0.0f;         // 目标速度（仅在速度模式下使用）
volatile int control_mode = 0;      // 控制模式：0 = 角度闭环，1 = 速度闭环
pid_controller_t manual_speed_pid;  // 专用速度PID

// 平滑上升速率（度/秒），用于防止目标角度突变导致大电流
const float MAX_RAMP_DEG_PER_SEC = 90.0f; // 90 deg/s as example
uint32_t last_pid_time = 0;  // 上次PID计算时间
uint8_t uart_tx_buffer[100];  // UART发送缓冲区
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
  MX_USART1_UART_Init();
  MX_CAN_Init();
  /* USER CODE BEGIN 2 */
  
  // 初始化串级PID控制器：外环角度PID、内环速度PID
  cascade_pid_init(&motor_cascade_pid,
                   2.0f,   0.005f, 0.02f,   // angle Kp, Ki, Kd
                   0.2f,   0.03f,  0.005f); // speed Kp, Ki, Kd

  // 初始目标（以输出轮角度为单位，度）
  desired_angle = 0.0f;
  motor_cascade_pid.target_angle = 0.0f;  // 转子角度
  motor_cascade_pid.target_speed = 0.0f;

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
    
    // 每10ms执行一次角度->速度->电流串级控制 (100Hz)
    if(current_time - last_pid_time >= 10)
    {
      // 关键保护：仅当电机反馈已初始化时才执行控制
      if (motor_angle_inited) 
      {
        // 计算时间间隔(秒)，最小防护防止极小时间间隔
        float dt = (current_time - last_pid_time) / 1000.0f;
        if (dt < 0.001f) dt = 0.001f;

        // 当前多圈角度与速度（考虑减速比）
        float current_angle = motor_continuous_angle / 19.2037f; // 单位：度（输出轴角度）
        float current_speed = (float)motor_feedback[0].speed / 19.2037f; // 单位：rpm（输出轴转速）

        // 平滑目标角度（防止突变）
        // desired_angle 是输出轮的目标角度，需要转换为转子角度
        float diff = desired_angle - (motor_cascade_pid.target_angle / 19.2037f);
        float max_step = MAX_RAMP_DEG_PER_SEC * dt;
        if (fabsf(diff) <= max_step) {
            motor_cascade_pid.target_angle = desired_angle * 19.2037f;
        } 
        else 
        {
            motor_cascade_pid.target_angle += (diff > 0 ? max_step : -max_step) * 19.2037f;
        }

        // 串级PID计算：输入目标角度（转子角度）、当前角度（输出轮角度）与当前速度（输出轮速度），返回输出电流
        // 注意：cascade_pid_calculate函数内部需要处理角度单位的转换
        float output_current = cascade_pid_calculate(&motor_cascade_pid, 
                                                   motor_cascade_pid.target_angle,     // 转子目标角度
                                                   current_angle,                      // 输出轮当前角度
                                                   current_speed,                      // 输出轮当前速度
                                                   dt);

        // 输出限幅与死区保护
        const float MAX_CURR = 12000.0f; // 根据电调能力调整，不能超过16384
        const float MIN_ACTIVE_CURR = 10.0f; // 小电流视为0
        if (output_current > MAX_CURR) 
        {
          output_current = MAX_CURR;
        }
        if (output_current < -MAX_CURR) 
        {
          output_current = -MAX_CURR;
        }
        if (fabsf(output_current) < MIN_ACTIVE_CURR) 
        {
          output_current = 0.0f;
        }

        // 调试 / CSV 输出： target_angle, current_angle, target_speed, current_speed, pid_out
        char debug_buffer[128];
        int len = snprintf(debug_buffer, sizeof(debug_buffer), "%.1f,%.1f,%.1f,%.1f,%.1f\r\n",
                           motor_cascade_pid.target_angle / 19.2037f,  // 输出轮目标角度
                           current_angle,  // 输出轮当前角度
                           motor_cascade_pid.target_speed / 19.2037f,  // 输出轮目标速度
                           current_speed,  // 输出轮当前速度
                           motor_cascade_pid.speed_pid.output);
        HAL_UART_Transmit(&huart1, (uint8_t*)debug_buffer, len, 50);

        // 发送电流命令
        can_dji_motor_control(1, (int16_t)output_current);
      } 
      else 
      {
        // 电机反馈未初始化，发送 0 电流保持电机空闲
        can_dji_motor_control(1, 0);
      }

      // 更新上次执行时间
      last_pid_time = current_time;
    }
    
    // 添加一个小延迟以减少CPU占用
    HAL_Delay(1);
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
