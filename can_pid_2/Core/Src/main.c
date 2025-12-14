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
#include "dma.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motor_control.h"
#include <stdio.h>
#include <string.h>
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
MotorControl_t motor1;

uint32_t last_control_time = 0;

// 串口发送数据缓冲区
typedef struct {
  uint8_t motor_id;
  uint16_t rotor_angle;
  int16_t rotor_speed;
  int16_t torque_current;
  int8_t temperature;
  uint8_t data_ready;  // 数据就绪标志
} UART_SendData_t;

static UART_SendData_t uart_send_data = {0};

/* * 修改说明：
 * 原 MotorFeedbackCallback 函数声明被移除/重命名。
 * 改为 Motor_Update_Status 以匹配 can.c 中的调用。
 */
void Motor_Update_Status(uint8_t motor_id, uint16_t rotor_angle, 
                           int16_t rotor_speed, int16_t torque_current, 
                           int8_t temperature);

// 串口发送CAN反馈数据函数
void UART_SendMotorFeedback(uint8_t motor_id, uint16_t rotor_angle, 
                            int16_t rotor_speed, int16_t torque_current, 
                            int8_t temperature);
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
  MX_DMA_Init();
  MX_CAN_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  MotorControl_Init(&motor1, 1);
  CAN_InitFilter(1);

  /* 修改说明：删除了 CAN_SetRxCallback 调用，改为直接硬链接 */
  // CAN_SetRxCallback(MotorFeedbackCallback); 
  
  CAN_StartReceive();
  MotorControl_SetEnable(&motor1, 1);
 
  motor1.target_output_angle = motor1.total_output_angle;
  
  //MotorControl_SetTargetAngle(&motor1, 180.0f);
  MotorControl_RotateTwoRoundsForward(&motor1);  // 正转两圈
  //MotorControl_RotateTwoRoundsReverse(&motor1);  // 反转两圈

  last_control_time = HAL_GetTick();
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    uint32_t current_time = HAL_GetTick();
    if (current_time - last_control_time >= 10)
    {

      int16_t current_command = MotorControl_Calculate(&motor1);
      
      CAN_SendMotorCommand(motor1.motor_id, current_command);
      
      last_control_time = current_time;
    }
    
    if (uart_send_data.data_ready)
    {
      uart_send_data.data_ready = 0;  // 清除标志
      UART_SendMotorFeedback(uart_send_data.motor_id,
                             uart_send_data.rotor_angle,
                             uart_send_data.rotor_speed,
                             uart_send_data.torque_current,
                             uart_send_data.temperature);
    }
    
    HAL_Delay(1);  // 避免CPU占用过高
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

/* * 修改说明：
 * 函数名从 MotorFeedbackCallback 改为 Motor_Update_Status。
 * 这个函数现在会被 can.c 中的 HAL_CAN_RxFifo0MsgPendingCallback -> CAN_RxCallback 直接调用。
 */
void Motor_Update_Status(uint8_t motor_id, uint16_t rotor_angle, int16_t rotor_speed, int16_t torque_current, int8_t temperature)
{
  if (motor_id == motor1.motor_id)
  {
    MotorControl_UpdateFeedback(&motor1, rotor_angle, rotor_speed, torque_current, temperature);
  }
  
  uart_send_data.motor_id = motor_id;
  uart_send_data.rotor_angle = rotor_angle;
  uart_send_data.rotor_speed = rotor_speed;
  uart_send_data.torque_current = torque_current;
  uart_send_data.temperature = temperature;
  uart_send_data.data_ready = 1;
 
}

void UART_SendMotorFeedback(uint8_t motor_id, uint16_t rotor_angle, int16_t rotor_speed, int16_t torque_current, int8_t temperature)
{
  char uart_buffer[200];
  int len;
  
  float total_output_angle = 0.0f;
  if (motor_id == motor1.motor_id)
  {
    total_output_angle = motor1.total_output_angle;
  }

  if (motor_id == motor1.motor_id)
  {
    len = snprintf(uart_buffer, sizeof(uart_buffer), 
                 "%.2f,%.2f,%.2f\r\n", 
                 total_output_angle,          // 通道1：总输出角度
                 motor1.target_output_angle,  // 通道2：目标角度
                 motor1.current_output_angle   // 通道3：当前角度
                 );
  }
  else
  {
    len = snprintf(uart_buffer, sizeof(uart_buffer), "0.00,0.00,0.00\r\n");
  }
  
  if (len > 0 && len < sizeof(uart_buffer))
  {
    HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, len, 100);
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
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  * where the assert_param error has occurred.
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