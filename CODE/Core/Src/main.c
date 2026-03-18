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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motor.h"
#include "servo.h"
#include "BlueTooth.h"
#include "string.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern uint8_t Bluetooth_RxData[Bluetooth_length];
extern struct Struct_Bluetooth_RxData RxData;
extern struct Struct_Bluetooth_PorcessData PorcessData;
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
#define MAX_PWM 100
#define DEAD_ZONE 0.1f
#define MAX_SPEED_SCALE 1.0f
#define   UNIT_PWM	0.1
// DMA接收缓冲区
#define UART_DMA_BUFFER_SIZE 256
uint8_t uart_dma_rx_buffer[UART_DMA_BUFFER_SIZE];
volatile uint16_t dma_received_length = 0;
volatile uint8_t dma_receive_complete = 0;
 typedef struct{
int16_t motor1_pwm;
int16_t motor2_pwm;
int16_t motor3_pwm;
int16_t motor4_pwm;
uint8_t data_valid;
}PWM_Data_t;//接收到的原始数据（未解包）
 
PWM_Data_t pwm_data;
typedef enum {
    MODE_MANUAL = 0,    // 手动模式（蓝牙/按键控制）
    MODE_AUTO = 1       // 自动模式（上位机控制）
} ControlMode_t;

volatile ControlMode_t current_mode = MODE_MANUAL;  // 默认手动模式
// 用于存储处理后的数据
uint8_t processed_data[UART_DMA_BUFFER_SIZE];
uint8_t Parse_Signed_PWM_Packet(uint8_t *data, uint16_t length);
void Con(void);
void move(void);
uint8_t count;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {
        Key_Scan();
	 
        //测试按键触发 单次按下只累加一次
        if(PorcessData.Key[0] == Key_Status_TRIG_FREE_PRESSED)
        {
          count ++;
        }
  
         if (huart1.ErrorCode)
        {
            HAL_UART_DMAStop(&huart1); // 停止以重启
            HAL_UARTEx_ReceiveToIdle_DMA( &huart1,Bluetooth_RxData,Bluetooth_length);
        }
				if (huart2.ErrorCode)
        {
            HAL_UART_DMAStop(&huart2); // 停止以重启
           HAL_UARTEx_ReceiveToIdle_DMA(&huart2,uart_dma_rx_buffer,UART_DMA_BUFFER_SIZE);
		
        }
    }
		
				
}
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART1)
    {
      if(Bluetooth_RxData[0] == 0xAA && Bluetooth_RxData[1] == 0x55)
      {
        Bluetooth_Data_Process(Bluetooth_RxData);
      }
      HAL_UARTEx_ReceiveToIdle_DMA(&huart1,Bluetooth_RxData,Bluetooth_length);
    }else if(huart->Instance == USART2){
		   dma_received_length=Size;
			dma_receive_complete=1;
		 HAL_UARTEx_ReceiveToIdle_DMA(&huart2,uart_dma_rx_buffer,UART_DMA_BUFFER_SIZE);
		
		}
		
		
		
}
void Process_Received_Data(void){
if(dma_receive_complete&&current_mode==MODE_AUTO){
memcpy(processed_data,uart_dma_rx_buffer,dma_received_length);
	Parse_Signed_PWM_Packet(processed_data,dma_received_length);
	dma_receive_complete=0;
}
 
}



void Con(void)
{
	Process_Received_Data();
    uint8_t unit_pwm =1 ; // 定义一个PWM值，根据需要调整
     
    // 使用按键控制机械臂动作
    
    // 夹爪控制
    if(PorcessData.Key[11] == Key_Status_TRIG_FREE_PRESSED|| 
        PorcessData.Key[11] == Key_Status_PRESSED) {
        // 夹爪闭合K2
        RobotArm_ShakeHand(unit_pwm); // 前舵机握手   
    }
    
    if(PorcessData.Key[10]   == Key_Status_TRIG_FREE_PRESSED|| 
        PorcessData.Key[10] == Key_Status_PRESSED) {
        // 夹爪打开K1
        RobotArm_LetHand(unit_pwm); // 前舵机松手 
    }
    
    if(PorcessData.Key[8] == Key_Status_TRIG_FREE_PRESSED|| 
        PorcessData.Key[8] == Key_Status_PRESSED  ) {
        RobotArm_RaiseHand(unit_pwm); // 大臂前伸，K4
    }
    
    if(PorcessData.Key[9] == Key_Status_TRIG_FREE_PRESSED|| 
        PorcessData.Key[9] == Key_Status_PRESSED) {
        RobotArm_DropHand(unit_pwm); // 大臂后缩，K3
    }
    
    if(PorcessData.Key[12] == Key_Status_PRESSED) {
        // k1
        RobotArm_WanDown(unit_pwm); // 手腕下弯R2
    }
    
    // 关节控制（使用按键持续按压）
    if(PorcessData.Key[13] == Key_Status_PRESSED) {
        // k2
        RobotArm_WanUp(unit_pwm); // 手腕上弯R1
    }
    
    if(PorcessData.Key[14] == Key_Status_TRIG_FREE_PRESSED || 
        PorcessData.Key[14] == Key_Status_PRESSED) {
        // k3
        RobotArm_StrechHand(unit_pwm); // 手肘抬头L2
    }
    
    if(PorcessData.Key[15] == Key_Status_TRIG_FREE_PRESSED || 
       PorcessData.Key[15] == Key_Status_PRESSED) {
        // k4
        RobotArm_ShinkHand(unit_pwm); // 手肘低头L1
    } 
		if(PorcessData.Key[0]==Key_Status_TRIG_FREE_PRESSED){//Y
		Servo_Init2();//左键恢复机械臂初始态
		}
		if(PorcessData.Key[1]==Key_Status_TRIG_FREE_PRESSED){//X
		Servo_Init();//左键恢复机械臂初始态
		}
	/*	if(PorcessData.Key[0]==Key_Status_TRIG_FREE_PRESSED){
			current_mode=MODE_AUTO;      
			memset(uart_dma_rx_buffer,0,UART_DMA_BUFFER_SIZE);
			dma_received_length=0; 
			dma_receive_complete = 0;
		HAL_UARTEx_ReceiveToIdle_DMA(&huart2,uart_dma_rx_buffer,UART_DMA_BUFFER_SIZE);
		__HAL_UART_ENABLE_IT(&huart2,UART_IT_IDLE);
			setspeed(0,0,0,0);
		}
		if(PorcessData.Key[1]==Key_Status_TRIG_FREE_PRESSED){//X
		current_mode=MODE_MANUAL;
		HAL_UART_DMAStop(&huart2);
			setspeed(0,0,0,0);
		}*/
}
void move(void){
if(PorcessData.Key[7] == Key_Status_TRIG_FREE_PRESSED|| 
       PorcessData.Key[7] == Key_Status_PRESSED) {
       setspeed(75,75,75,75); 
    }
else if(PorcessData.Key[6] == Key_Status_TRIG_FREE_PRESSED|| 
       PorcessData.Key[6] == Key_Status_PRESSED) {
       setspeed(-75,-75,-75,-75);
    }  
else if(PorcessData.Key[4] == Key_Status_TRIG_FREE_PRESSED|| 
       PorcessData.Key[4] == Key_Status_PRESSED 
       ) {
       setspeed(-75,70,90,-90);    
    }else if(PorcessData.Key[5] == Key_Status_TRIG_FREE_PRESSED|| 
       PorcessData.Key[5] == Key_Status_PRESSED ) {//OK
       setspeed(70,-65 ,-78,78 );
    }else {
		setspeed(0,0,0,0); 
		}


if(PorcessData.Remote_Right_X>0){
	
setspeed(-75,75,-75,75);
}
if(PorcessData.Remote_Right_X<0){
	
setspeed(75,-75,75,-75);
}

if(PorcessData.Remote_Left_X||PorcessData.Remote_Left_Y){
float move_x=PorcessData.Remote_Left_Y;
float move_y=PorcessData.Remote_Left_X;
float rotate=PorcessData.Remote_Right_X;
	if(fabsf(move_x)<DEAD_ZONE)move_x=0;//死区设置
	if(fabsf(move_y)<DEAD_ZONE)move_y=0;
	if(fabsf(rotate)<DEAD_ZONE)rotate=0;
float wheel1_speed = (move_x - move_y )* MAX_PWM;   // 左前轮
float wheel2_speed = (move_x + move_y )*MAX_PWM;  // 右前轮   
float wheel3_speed = (move_x + move_y )*MAX_PWM;  // 左后轮
float wheel4_speed = (move_x - move_y )*MAX_PWM;   // 右后轮
    //wheel_speed范围-99.0-99.0
	int16_t pwm_1 = (int16_t)wheel1_speed;
	int16_t pwm_2 = (int16_t)wheel2_speed;
	int16_t pwm_3 = (int16_t)wheel3_speed;
	int16_t pwm_4 = (int16_t)wheel4_speed;
    if (pwm_1 > 75) pwm_1 =75;
    if (pwm_1 < -75) pwm_1 = -75;
	  if (pwm_2 > 75) pwm_2 = 75;
    if (pwm_2 < -75) pwm_2 = -75;
	  if (pwm_3 > 75) pwm_3 = 75;
    if (pwm_3 < -75) pwm_3 = -75;
	  if (pwm_4 > 75) pwm_4 = 75;
    if (pwm_4 < -75) pwm_4 = -75; 
	setspeed(pwm_1,pwm_2,pwm_3,pwm_4);
	
}

}
 
  
uint8_t Parse_Signed_PWM_Packet(uint8_t *data, uint16_t length)
{
    uint16_t packet_count = 0;
    uint16_t i = 0;
    if(current_mode!=MODE_AUTO){
		return 0;
		}
    while (i < length) {
        if (data[i] == 0xAA && (i + 9) < length) {
            if (data[i + 9] == 0x55) {
                // 将有符号字节转换为有符号16位整数
                int16_t motor1_pwm = (int16_t)((data[i + 1] << 8) | data[i + 2]);
                int16_t motor2_pwm = (int16_t)((data[i + 3] << 8) | data[i + 4]);
                int16_t motor3_pwm = (int16_t)((data[i + 5] << 8) | data[i + 6]);
                int16_t motor4_pwm = (int16_t)((data[i + 7] << 8) | data[i + 8]);
                
                pwm_data.motor1_pwm = motor1_pwm;
                pwm_data.motor2_pwm = motor2_pwm;
                pwm_data.motor3_pwm = motor3_pwm;
                pwm_data.motor4_pwm = motor4_pwm;
                
                // 数据有效性检查（根据你的PWM范围调整）
                if (motor1_pwm >= -99 && motor1_pwm <= 99 &&
                    motor2_pwm >= -99 && motor2_pwm <= 99 &&
                    motor3_pwm >= -99 && motor3_pwm <= 99 &&
                    motor4_pwm >= -99 && motor4_pwm <= 99) {
                    pwm_data.data_valid = 1;
                    packet_count++;
                    
                    // 处理数据
                   if(current_mode==MODE_AUTO){ setspeed(motor1_pwm,motor2_pwm,motor3_pwm,motor4_pwm);
									 }
									 i += 10;
                    continue;
                }
            }
        }
        i++;
    } 
    
    return (packet_count > 0);
}




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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
HAL_TIM_Base_Start_IT(&htim2);
HAL_UARTEx_ReceiveToIdle_DMA(&huart1,Bluetooth_RxData,Bluetooth_length);
HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
//HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_4);
//HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3);
HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_4);
HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);
 
Servo_Init();
  /* USER CODE END 2 */
 
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
Con();
HAL_Delay(10);
move();
HAL_Delay(10);
		
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
