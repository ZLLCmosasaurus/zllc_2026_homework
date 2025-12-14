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
#include "dma.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define _MIN 364
#define _MAX 1684
#define _MID 1024
#define _DEN (_MAX-_MID)
#define CONSTRAIN(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))

extern DMA_HandleTypeDef hdma_usart3_rx;
typedef enum {
  SW_UP    = 1,
  SW_MID   = 2,
  SW_DOWN  = 3 
} Switch_State;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
typedef struct 
{
  uint16_t ch0;
  uint16_t ch1;
  uint16_t ch2;
  uint16_t ch3;
  
  uint8_t s1;
  uint8_t s2;
	Switch_State s1_state;
  Switch_State s2_state;
	
	float _ch0;
	float _ch1;
	float _ch2;
  float _ch3;
	
}RC_Ctl_t;

RC_Ctl_t RC_Ctrl={0};

uint8_t buffer[50]={0};

uint16_t rx_len = 0;
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
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
HAL_UARTEx_ReceiveToIdle_DMA(&huart3,buffer,18);
//__HAL_DMA_DISABLE_IT(&hdma_usart3_rx,DMA_IT_HT);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart,uint16_t Size)
{
  if(huart ->Instance == USART3)
  { 
		//rx_len = Size;
    //if (rx_len == 18){
    RC_Ctrl.ch0 = ((int16_t)buffer[0] | ((int16_t)buffer[1] << 8)) & 0x07FF;  
    RC_Ctrl.ch1 = (((int16_t)buffer[1] >> 3) | ((int16_t)buffer[2] << 5)) 
& 0x07FF; 
    RC_Ctrl.ch2 = (((int16_t)buffer[2] >> 6) | ((int16_t)buffer[3] << 2) | 
                         ((int16_t)buffer[4] << 10)) & 0x07FF; 
    RC_Ctrl.ch3 = (((int16_t)buffer[4] >> 1) | ((int16_t)buffer[5]<<7)) & 
0x07FF; 
     
    RC_Ctrl.s1 = ((buffer[5] >> 4) & 0x000C) >> 2; 
    RC_Ctrl.s2 = ((buffer[5] >> 4) & 0x0003);

		
		uint16_t ch0_constrain = CONSTRAIN(RC_Ctrl.ch0, _MIN, _MAX);
    uint16_t ch1_constrain = CONSTRAIN(RC_Ctrl.ch1, _MIN, _MAX);
    uint16_t ch2_constrain = CONSTRAIN(RC_Ctrl.ch2, _MIN, _MAX);
    uint16_t ch3_constrain = CONSTRAIN(RC_Ctrl.ch3, _MIN, _MAX);
		
		RC_Ctrl._ch0 = (float)(ch0_constrain - _MID) / _DEN;
    RC_Ctrl._ch1 = (float)(ch1_constrain - _MID) / _DEN;
    RC_Ctrl._ch2 = (float)(ch2_constrain - _MID) / _DEN;
    RC_Ctrl._ch3 = (float)(ch3_constrain - _MID) / _DEN;
		
		if(RC_Ctrl.s1 == 1)       RC_Ctrl.s1_state = SW_UP;
    else if(RC_Ctrl.s1 == 2)  RC_Ctrl.s1_state = SW_MID;
    else                      RC_Ctrl.s1_state = SW_DOWN;
    
    if(RC_Ctrl.s2 == 1)       RC_Ctrl.s2_state = SW_UP;
    else if(RC_Ctrl.s2 == 2)  RC_Ctrl.s2_state = SW_MID;
    else                      RC_Ctrl.s2_state = SW_DOWN;
    
    //__HAL_DMA_DISABLE_IT(&hdma_usart3_rx,DMA_IT_HT);
//	  }else
//		{ 
//		memset(buffer, 0, sizeof(buffer));
		
		HAL_UARTEx_ReceiveToIdle_DMA(&huart3,buffer,18);
  }
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3)
    {
        if (huart->ErrorCode & HAL_UART_ERROR_FE)
        {
            // ???????
            __HAL_UART_CLEAR_FLAG(huart, UART_FLAG_FE);
            // ??????
        	huart->RxState = HAL_UART_STATE_READY;
            // ????????
            __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
        }
        
        // ???? DMA ??
       HAL_UARTEx_ReceiveToIdle_DMA(&huart3,buffer,18);
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
