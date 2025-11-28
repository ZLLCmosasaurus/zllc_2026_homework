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
#include "gpio.h"

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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t flag=1;
float move,move_sum;
uint8_t tx_data[8]={1,2,3,4,5,6,7,8};
uint8_t rx_data[8]={0};
int16_t change=1;
int16_t target_angle=360;
struct Speed
{
  int16_t speed;
  volatile float Kp, Ki, Kd, Real, Out, Err0, Err1, ErrInt;
  volatile float Want;
};
struct Angle
{
  float angle,lastangle;
  volatile float Kp, Ki, Kd, Real, Out, Err0, Err1, ErrInt;
  volatile float Want;
};

struct Speed Speed_ctrl = {
  .speed = 0,
  .Kp = 8.4,
  .Ki = 1.2,
  .Kd = 30,
  .Real = 0,
  .Out = 0, 
  .Err0 = 0,
  .Err1 = 0,
  .ErrInt = 0,
  .Want = 0
};

struct Angle Angle_ctrl = {
  .angle = 0,
  .lastangle = 0,
  .Kp = 10,
  .Ki = 0,
  .Kd = 5,
  .Real = 0,
  .Out = 0, 
  .Err0 = 0,
  .Err1 = 0,
  .ErrInt = 0,
  .Want = 0
};

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
  MX_CAN_Init();
  /* USER CODE BEGIN 2 */
  // 在CAN初始化后启用FIFO0中断
if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
{
    Error_Handler();
}
if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO1_MSG_PENDING) != HAL_OK)
{
    Error_Handler();
}
  /* USER CODE END 2 */
  HAL_Delay(100);
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  Angle_ctrl.Want=-720;
  while (1)
  {
    // if(Angle_ctrl.lastangle-Angle_ctrl.angle<-350)
    // {
    //   Angle_ctrl.Want+=360;
    // }

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
//FIFO0接收中断回调
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  if(My_CAN_Receive(rx_data)==8)
  {
    //外环角度环
    Angle_ctrl.lastangle=Angle_ctrl.angle;
    Angle_ctrl.angle=(float)(rx_data[0]<<8|rx_data[1]);
    Angle_ctrl.angle=(Angle_ctrl.angle*360.0)/8191.0;
    move=Angle_ctrl.angle-Angle_ctrl.lastangle;
    if(move<=-350)
    {
      move+=360;
    }
    if(move>=350)
    {
      move-=360;
    }
    move_sum+=move;
		//获取实际值
    Angle_ctrl.Real= move_sum;
		Angle_ctrl.Err0= Angle_ctrl.Err1;
		Angle_ctrl.Err1= Angle_ctrl.Want- Angle_ctrl.Real;
		Angle_ctrl.ErrInt +=  Angle_ctrl.Err0;
    //积分限幅
    //...
		 Angle_ctrl.Out= Angle_ctrl.Kp* Angle_ctrl.Err1+ Angle_ctrl.Ki* Angle_ctrl.ErrInt+ Angle_ctrl.Kd*( Angle_ctrl.Err1- Angle_ctrl.Err0);
		//输出限幅
		if( Angle_ctrl.Out>=200)
		{
			 Angle_ctrl.Out=200;
		}
		if( Angle_ctrl.Out<=-200)
		{
			 Angle_ctrl.Out=-200;
		}
		//把Out输出，进行调控
    // if(Angle_ctrl.lastangle-Angle_ctrl.angle>350)
    // {
    //   Angle_ctrl.angle=360;
    // }
    Speed_ctrl.Want=Angle_ctrl.Out;

    //内环速度环
    Speed_ctrl.speed=(int16_t)(rx_data[2]<<8|rx_data[3]);
		//获取实际值
    Speed_ctrl.Real=Speed_ctrl.speed;
		Speed_ctrl.Err0=Speed_ctrl.Err1;
		Speed_ctrl.Err1=Speed_ctrl.Want-Speed_ctrl.Real;
		Speed_ctrl.ErrInt += Speed_ctrl.Err0;
    //积分限幅
    if(Speed_ctrl.ErrInt>=100)
    {
      Speed_ctrl.ErrInt=100;
    }
    if(Speed_ctrl.ErrInt<=-100)
    {
      Speed_ctrl.ErrInt=-100;
    }
		Speed_ctrl.Out=Speed_ctrl.Kp*Speed_ctrl.Err1+Speed_ctrl.Ki*Speed_ctrl.ErrInt+Speed_ctrl.Kd*(Speed_ctrl.Err1-Speed_ctrl.Err0);
		//输出限幅
		if( Angle_ctrl.Out>=100)
		{
			 Angle_ctrl.Out=100;
		}
		if( Angle_ctrl.Out<=-100)
		{
			 Angle_ctrl.Out=-100;
		}
		My_CAN_Sete((int16_t) Speed_ctrl.Out);
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
