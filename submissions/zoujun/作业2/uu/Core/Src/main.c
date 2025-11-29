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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "SEGGER_RTT.h"
#include "motor_simple.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern CAN_HandleTypeDef hcan;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// ������Ʊ�������motor_simple.c�ж��壬��������Ϊextern��
extern int16_t target_rpm;
extern int16_t actual_rpm;
extern uint8_t motor_online;
extern uint32_t last_motor_time;
// ȫ�ֱ�������
uint32_t g_can_rx_count = 0;
uint32_t g_can_tx_count = 0;
uint32_t g_last_control_time = 0;
uint32_t g_last_display_time = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void CAN_Filter_Config(void)
{
  CAN_FilterTypeDef can_filter;

  /* ����CAN�˲��� - �������е������֡ (0x201-0x204) */
  can_filter.FilterBank = 0;
  can_filter.FilterMode = CAN_FILTERMODE_IDMASK;
  can_filter.FilterScale = CAN_FILTERSCALE_32BIT;
  can_filter.FilterIdHigh = 0x0000; // ID��16λ
  can_filter.FilterIdLow = 0x0000;
  can_filter.FilterMaskIdHigh = 0x0000; // ����λ��16λ
  can_filter.FilterMaskIdLow = 0x0000;
  can_filter.FilterFIFOAssignment = CAN_RX_FIFO0;
  can_filter.FilterActivation = ENABLE;
	can_filter.SlaveStartFilterBank = 14;

  if (HAL_CAN_ConfigFilter(&hcan, &can_filter) != HAL_OK)
  {
    Error_Handler();
  }
	HAL_CAN_Start(&hcan);
  HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
}
void force_current_test(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// CAN����ص�
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
    uint32_t error_code = HAL_CAN_GetError(hcan);
    SEGGER_RTT_printf(0, "CAN Error: 0x%08lX\n", error_code);
}

// CAN�����ж�
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t data[8];
    
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, data) == HAL_OK) {
        g_can_rx_count++;
        
        // ��������������
        motor_process_data(rx_header.StdId, data);
    }
}
// ǿ�Ƶ�������
void force_current_test(void)
{
    SEGGER_RTT_WriteString(0, "=== FORCE CURRENT TEST START ===\n");
    
    // ����������
    for(int i = 0; i < 3; i++) {
        send_current_to_motor(2000);
        HAL_Delay(500);
    }
    
    // ���Ը�����
    for(int i = 0; i < 3; i++) {
        send_current_to_motor(-2000);
        HAL_Delay(500);
    }
    
    // ֹͣ
    send_current_to_motor(0);
    SEGGER_RTT_WriteString(0, "=== FORCE CURRENT TEST END ===\n\n");
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2) {
        // 1ms PID��������
        update_pid_control();
    }
}
// �˲������
#define CAN_FILTER(x) ((x) << 3)

// ���ն���
#define CAN_FIFO_0 (0 << 2)
#define CAN_FIFO_1 (1 << 2)

//��׼֡����չ֡
#define CAN_STDID (0 << 1)
#define CAN_EXTID (1 << 1)

// ����֡��ң��֡
#define CAN_DATA_TYPE    0
#define CAN_REMOTE_TYPE  1
/**
 * @brief ����CAN�Ĺ�����
 *
 * @param hcan CAN���
 * @param Object_Para ɸѡ�����0-27 | FIFOx | ID���� | ֡����
 * @param ID ��֤��
 * @param Mask_ID ������(0x3ff, 0x1fffffff)
 */
void can_filter_mask_config(CAN_HandleTypeDef *hcan, uint8_t Object_Para, uint32_t ID, uint32_t Mask_ID)
{
	
    //��⴫���Ƿ���ȷ
    assert_param(hcan != NULL);

	   //CAN��������ʼ���ṹ��
    CAN_FilterTypeDef can_filter_init_structure;
    //�˲������, 0-27, ��28���˲���
    can_filter_init_structure.FilterBank = Object_Para >> 3;
    //�˲���ģʽ������ID����ģʽ
    can_filter_init_structure.FilterMode = CAN_FILTERMODE_IDMASK;
    
	
    if ((Object_Para & 0x02))
    {   
        //29λ ��չ֡
			  // 32λ�˲�
        can_filter_init_structure.FilterScale = CAN_FILTERSCALE_32BIT;
        //��֤�� ��16bit
        can_filter_init_structure.FilterIdHigh = (ID << 3) >> 16;
        //��֤�� ��16bit
        can_filter_init_structure.FilterIdLow = ID << 3 | (Object_Para & 0x03) << 1;
        //������ ��16bit
        can_filter_init_structure.FilterMaskIdHigh = (Mask_ID << 3) >> 16;
        //������ ��16bit
        can_filter_init_structure.FilterMaskIdLow = Mask_ID << 3 | (0x03) << 1 ;
    }
    else
    {
        //11λ ��׼֡
			  // 32λ�˲�
        can_filter_init_structure.FilterScale = CAN_FILTERSCALE_16BIT;
        //��׼֡��֤�� ��16bit������
        can_filter_init_structure.FilterIdHigh = 0x0000 ; 
        //��֤�� ��16bit
			  can_filter_init_structure.FilterIdLow =ID << 5 | (Object_Para & 0x02) << 4;  
        //��׼֡������ ��16bit������
        can_filter_init_structure.FilterMaskIdHigh =  0x0000 ;
        //������ ��16bit
        can_filter_init_structure.FilterMaskIdLow =(Mask_ID << 5) | 0x01 << 4 ; 
    }

    //�˲�����FIFO0��FIFO1
    can_filter_init_structure.FilterFIFOAssignment = (Object_Para >> 2) & 0x01;
    //�ӻ�ģʽѡ��ʼ��Ԫ , ǰ14����CAN1, ��14����CAN2
    can_filter_init_structure.SlaveStartFilterBank = 14;
    //ʹ���˲���
    can_filter_init_structure.FilterActivation = ENABLE;

    // ����������
    if(HAL_CAN_ConfigFilter(hcan, &can_filter_init_structure)!=HAL_OK)
    {
        Error_Handler();
    }
	
}

uint8_t CAN_Send_Data(CAN_HandleTypeDef *hcan, uint16_t ID, uint8_t *Data, uint16_t Length)
{
    CAN_TxHeaderTypeDef tx_header;
    uint32_t used_mailbox;

    //检测传参是否正确
    assert_param(hcan != NULL);

    tx_header.StdId = ID;
    tx_header.ExtId = 0;
    tx_header.IDE = 0;
    tx_header.RTR = 0;
    tx_header.DLC = Length;

    return (HAL_CAN_AddTxMessage(hcan, &tx_header, Data, &used_mailbox));
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
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  //CAN_Filter_Config();
	can_filter_mask_config(&hcan, CAN_FILTER(0) | CAN_FIFO_0 | CAN_STDID | CAN_DATA_TYPE, 0 ,0);
	can_filter_mask_config(&hcan, CAN_FILTER(1) | CAN_FIFO_1 | CAN_STDID | CAN_DATA_TYPE, 0 ,0);
	    /*离开初始模式*/
    HAL_CAN_Start(&hcan);				
    
    /*开中断*/
    HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);       //can 接收fifo 0不为空中断
		
	HAL_TIM_Base_Start_IT(&htim2);
	// ������Ϣ
  SEGGER_RTT_WriteString(0, "=========================================\r\n");
  SEGGER_RTT_WriteString(0, "  C620 + M3508 Motor Control Started!\r\n");
  SEGGER_RTT_WriteString(0, "  CAN: 1Mbps, ID: 0x200/0x201\r\n");
  SEGGER_RTT_WriteString(0, "  PID Control: 1kHz (TIM2 Interrupt)\r\n");
  SEGGER_RTT_WriteString(0, "=========================================\r\n");
  SEGGER_RTT_WriteString(0, "Control: Change 'target_rpm' in Ozone Watch\r\n");
  SEGGER_RTT_WriteString(0, "Default: target_rpm = 500\r\n");
  SEGGER_RTT_WriteString(0, "=========================================\r\n\r\n");

  // ��ʼ���������
  send_current_to_motor(0);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		// ����PID����
//  uint32_t current_time = HAL_GetTick();
//    
//    // ���������״̬
//    if (current_time - last_motor_time > 200) {
//        motor_online = 0;
//    }
//    
//    // ÿ500ms��ʾ״̬
//    if (current_time - g_last_display_time > 500) {
//        if (motor_online) {
//            SEGGER_RTT_printf(0, "STATUS: Target=%d, Actual=%d, Online=%d\n", 
//                             target_rpm, actual_rpm, motor_online);
//        } else {
//            SEGGER_RTT_WriteString(0, "STATUS: Motor OFFLINE - No feedback\n");
//        }
//        SEGGER_RTT_printf(0, "CAN: RX=%lu, TX=%lu\n\n", g_can_rx_count, g_can_tx_count);
//        g_last_display_time = current_time;
//    }
//		 HAL_Delay(10); 
//  }
uint8_t data[8] = {0,1,2,0};
		//CAN_Send_Data(&hcan,0x201,data,8);
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
