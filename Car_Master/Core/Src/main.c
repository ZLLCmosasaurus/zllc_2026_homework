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
#include "SerialCom.h"
#include <stdio.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct{
    uint8_t header[2];//0xAA 0x55
    uint8_t length;
    uint8_t frame_id;
    int8_t left_x;
    int8_t left_y;
    int8_t right_x;
    int8_t right_y;
    uint8_t keys1;
    uint8_t keys2;
    uint8_t checksum;
}XProtocol_Handle_t;

typedef struct {
    uint8_t header[2];
    uint8_t length;
    int16_t rpm[4];
    uint8_t checksum;
} Encoder_Recv_t;

typedef enum{
    STATE_HEADER1,
    STATE_HEADER2,
    STATE_LENGTH,
    STATE_DATA,
}ParserState_t;

typedef struct{
    int32_t target_rpm;
    int32_t current_rpm;
    int32_t encoder_last;
    int32_t pwm_output;
    int32_t integral;
    int32_t error_last;
}Motor_t;

Motor_t motor[4];
uint16_t servo_pwm[4]={1500,1000,2500,500};

ParserState_t bluetooth_parser_state=STATE_HEADER1;
XProtocol_Handle_t  rc_data;
uint8_t bluetooth_rx_buffer[11];
uint8_t data_buffer[16];
uint8_t bluetooth_rx_index=0;
uint8_t new_data_ready=0;
Encoder_Recv_t encoder_recv;
uint8_t encoder_rx_buf[12]; // 接收缓冲区（12字节）
uint8_t encoder_rx_idx = 0;

uint32_t motor_channel[4]={TIM_CHANNEL_1,TIM_CHANNEL_2,TIM_CHANNEL_3,TIM_CHANNEL_4};
uint32_t servo_channel[4]={TIM_CHANNEL_1,TIM_CHANNEL_2,TIM_CHANNEL_3,TIM_CHANNEL_4};
void parce_rc_command(void);
int32_t PID_Calculate(int32_t target,int32_t current,Motor_t* m);
void set_motor_speed(int motor_id,int32_t pwm);
void emergency_stop(void);
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
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
HAL_UART_Receive_IT(&huart1,bluetooth_rx_buffer,1);
HAL_UART_Receive_IT(&huart2, &rx_data, 1);
HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3);
HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_4);
HAL_TIM_Base_Start_IT(&htim4);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  
  while (1)
  {
    
    
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


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
    if(huart->Instance==USART1){
    uint8_t byte=bluetooth_rx_buffer[0];
        switch(bluetooth_parser_state){
            case STATE_HEADER1:
                if(byte==0xAA){
                    rc_data.header[0]=byte;
                    bluetooth_parser_state=STATE_HEADER2;
                }
                break;
                
            case STATE_HEADER2:
                if(byte==0x55)
                {
                    rc_data.header[1]=byte;
                    bluetooth_parser_state=STATE_LENGTH;
                }
                else{
                    bluetooth_parser_state=STATE_HEADER1;
                }
                break;
            case STATE_LENGTH:
                if(byte==0x0B){
                    rc_data.length=byte;
                     bluetooth_rx_index=0;
                    bluetooth_parser_state=STATE_DATA;
                }
                else{
                    bluetooth_parser_state=STATE_HEADER1;
                }
                break;
            
            case STATE_DATA:
                data_buffer[bluetooth_rx_index]=byte;
                bluetooth_rx_index++;
            
                if(bluetooth_rx_index>=9){
                    rc_data.frame_id=data_buffer[0];
                    rc_data.left_x=(int8_t)data_buffer[1];
                    rc_data.left_y=(int8_t)data_buffer[2];
                    rc_data.right_x=(int8_t)data_buffer[3];
                    rc_data.right_y=(int8_t)data_buffer[4];
                    rc_data.keys1=data_buffer[5];
                    rc_data.keys2=data_buffer[6];
                    rc_data.checksum=data_buffer[7];
                   if(rc_data.left_y>100)rc_data.left_y=100;
                    if(rc_data.left_y<-100)rc_data.left_y=-100;
                    if(rc_data.right_x>100)rc_data.right_x=100;
                    if(rc_data.right_x<-100)rc_data.right_x=-100;
                
                    uint8_t calc_sum=rc_data.header[0]+rc_data.header[1]+rc_data.length+rc_data.frame_id+(uint8_t)rc_data.left_x+(uint8_t)rc_data.left_y+
                    (uint8_t)rc_data.right_x+(uint8_t)rc_data.right_y+rc_data.keys1+rc_data.keys2;
                   if(calc_sum==rc_data.checksum){
                        parce_rc_command();
                       new_data_ready=1;
                   }
                    bluetooth_parser_state=STATE_HEADER1;
                }
                break;
            }
        HAL_UART_Receive_IT(&huart1,bluetooth_rx_buffer,1);
        }
else if(huart->Instance == USART2) {
        encoder_rx_buf[encoder_rx_idx++] = rx_data;
        if(encoder_rx_idx >= 12) {
            memcpy(&encoder_recv, encoder_rx_buf, 12);           
            uint8_t calc_sum = 0;
            for(uint8_t i=0; i<11; i++) calc_sum += encoder_rx_buf[i];
            
            if(encoder_recv.header[0] == 0xAA && encoder_recv.header[1] == 0x55 
               && encoder_recv.checksum == calc_sum && encoder_recv.length == 8) {
                
                for(uint8_t i=0; i<4; i++) {
                    motor[i].current_rpm = encoder_recv.rpm[i];
                }
            }
            encoder_rx_idx = 0; 
        }
        HAL_UART_Receive_IT(&huart2, &rx_data, 1); 
    }
}


void parce_rc_command(void){
    char debug_msg[60];
    
    if(rc_data.left_y == 0 && rc_data.right_x == 0) {
        for(int i = 0; i < 4; i++) {
            motor[i].target_rpm = 0;
            motor[i].integral = 0;
            motor[i].error_last = 0;  // 重置微分项
        }
    }
    
    int32_t move_speed=rc_data.left_y*8;
    int32_t turn_speed=rc_data.right_x*5;
    
    motor[0].target_rpm=move_speed-turn_speed;//左前
    motor[1].target_rpm=move_speed+turn_speed;//右前
    motor[2].target_rpm=move_speed-turn_speed;//左后
    motor[3].target_rpm=move_speed+turn_speed;//右后
    
    for(int i=0;i<4;i++){
        if(motor[i].target_rpm>1000)motor[i].target_rpm=1000;
        if(motor[i].target_rpm<-1000)motor[i].target_rpm=-1000;
    }
    if(rc_data.keys1!=0x00){
    //舵机1、2
    if(rc_data.keys1&0x08){
        servo_pwm[0]+=100;
        if(servo_pwm[0]>2500)servo_pwm[0]=2500;
    }
    if(rc_data.keys1&0x04){
        servo_pwm[0]-=100;
        if(servo_pwm[0]<500)servo_pwm[0]=500;
    }
    if(rc_data.keys1&0x02){
        servo_pwm[1]+=100;
        if(servo_pwm[1]>2500)servo_pwm[1]=2500;
    }
    if(rc_data.keys1&0x01){
        servo_pwm[1]-=100;
        if(servo_pwm[1]<200)servo_pwm[1]=500;
    }
//舵机3、4
    if(rc_data.keys1&0x80){
        servo_pwm[2]+=100;
        if(servo_pwm[2]>2500)servo_pwm[2]=2500;
    }
    if(rc_data.keys1&0x40){
        servo_pwm[2]-=100;
        if(servo_pwm[2]<500)servo_pwm[2]=500;
    }
    if(rc_data.keys1&0x20){
        servo_pwm[3]+=100;
        if(servo_pwm[3]>2500)servo_pwm[3]=2500;
    }
    if(rc_data.keys1&0x10){
        servo_pwm[3]-=100;
        if(servo_pwm[3]<500)servo_pwm[3]=500;
    }
    //更新舵机PWM
    for(int i=0;i<4;i++){
     __HAL_TIM_SET_COMPARE(&htim1,servo_channel[i],servo_pwm[i]);
    }
     __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,servo_pwm[1]);
    __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,servo_pwm[2]);
    
}
}
int32_t PID_Calculate(int32_t target,int32_t current,Motor_t* m){
    int32_t error=target-current;
    if(target==0){
        m->integral=0;
       m->error_last = 0;
        return 0;
    }
    else{
    m->integral+=error;
        if(m->integral>1000)m->integral=1000;
        if(m->integral<-1000)m->integral=-1000;
    }
    int32_t D=error-m->error_last;
    m->error_last=error;
    int32_t output=15*error+2*m->integral+8*D;
    if(output>1000)output=1000;
    if(output<-1000)output=-1000;
    return output;
}

void set_motor_speed(int motor_id,int32_t pwm){
    GPIO_PinState in1,in2;
    if(pwm>0){
        in1=GPIO_PIN_SET;in2=GPIO_PIN_RESET;
        
    }
    else if(pwm<0){
        in1=GPIO_PIN_RESET;in2=GPIO_PIN_SET;
        pwm=-pwm;
        
    }
    else{
        in1=GPIO_PIN_RESET;in2=GPIO_PIN_RESET;
        
    }
    switch(motor_id){
        case 0:
        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,in1);
        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,in2);
        __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,pwm);
        break;
    
        case 1:
        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,in1);
        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,in2);
        __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,pwm);
        break;
    
        case 2:
        HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,in1);
        HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,in2);
        __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,pwm);
        break;
   
        case 3:
        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,in1);
        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,in2);
        __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_4,pwm);
        break;
    }

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
    if(htim->Instance==TIM4){
       
        
        for(int i=0;i<4;i++){
            
            motor[i].pwm_output=PID_Calculate(motor[i].target_rpm,motor[i].current_rpm,&motor[i]);
            set_motor_speed(i,motor[i].pwm_output);
        }
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
