/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  * of the CAN instances.
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "can.h"

/* USER CODE BEGIN 0 */
#include "motor_control.h"

/**
 * @brief 外部定义的电机状态更新函数
 * @note  需要在 main.c 或 motor_control.c 中实现此函数，
 * 用于根据 motor_id 找到对应的电机实例并更新反馈数据。
 */
extern void Motor_Update_Status(uint8_t motor_id, uint16_t rotor_angle, 
                                int16_t rotor_speed, int16_t torque_current, 
                                int8_t temperature);

/* USER CODE END 0 */

CAN_HandleTypeDef hcan;

/* CAN init function */
void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 4;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_2TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_3TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_5TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**CAN GPIO Configuration
    PA11     ------> CAN_RX
    PA12     ------> CAN_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(USB_HP_CAN1_TX_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USB_HP_CAN1_TX_IRQn);
    HAL_NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();

    /**CAN GPIO Configuration
    PA11     ------> CAN_RX
    PA12     ------> CAN_TX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USB_HP_CAN1_TX_IRQn);
    HAL_NVIC_DisableIRQ(USB_LP_CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/* 已删除 CAN_SetRxCallback 函数 */

HAL_StatusTypeDef CAN_InitFilter(uint8_t motor_id)
{
    CAN_FilterTypeDef sFilterConfig;
    
    sFilterConfig.FilterBank = 0;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterIdHigh = 0x00000000;
    sFilterConfig.FilterIdLow = 0x00000000;
    sFilterConfig.FilterMaskIdHigh = 0x00000000;
    sFilterConfig.FilterMaskIdLow = 0x00000000;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14;
    
    if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK)
    {
        return HAL_ERROR;
    }
    
    return HAL_OK;
}

void CAN_StartReceive(void)
{
    // 启动CAN
    if (HAL_CAN_Start(&hcan) != HAL_OK)
    {
        Error_Handler();
    }
    
    HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
}

HAL_StatusTypeDef CAN_SendESCCommands(int16_t currents[4])
{
    CAN_TxHeaderTypeDef TxHeader;
    uint8_t TxData[8];
    uint32_t TxMailbox;
    HAL_StatusTypeDef status;
    int i;
    
    TxHeader.StdId = 0x200;
    TxHeader.ExtId = 0x01;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.DLC = 8;
    TxHeader.TransmitGlobalTime = DISABLE;
    
    for (i = 0; i < 4; i++)
    {
        if (currents[i] > 16384) 
        {
            currents[i] = 16384;
        }
        else if (currents[i] < -16384) 
        {
            currents[i] = -16384;
        }

        TxData[i * 2] = (currents[i] >> 8) & 0xFF;      // 高8位
        TxData[i * 2 + 1] = currents[i] & 0xFF;         // 低8位
    }
    
    status = HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox);
    
    return status;
}

HAL_StatusTypeDef CAN_SendMotorCommand(uint8_t motor_id, int16_t current)
{
    int16_t currents[4] = {0, 0, 0, 0};
    
    if (motor_id >= 1 && motor_id <= 4)
    {
        currents[motor_id - 1] = current;
    }
    
    return CAN_SendESCCommands(currents);
}

void CAN_RxCallback(void)
{
    CAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[8];
    uint8_t motor_id;
    uint16_t rotor_angle;
    int16_t rotor_speed;
    int16_t torque_current;
    int8_t temperature;
    
    if (HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0) > 0)
    {
        if (HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
        {
            if (RxHeader.StdId >= 0x200 && RxHeader.StdId <= 0x203)
            {
                motor_id = RxHeader.StdId - 0x200;
                
                rotor_angle = (RxData[0] << 8) | RxData[1];                 // 转子机械角度
                rotor_speed = (int16_t)((RxData[2] << 8) | RxData[3]);      // 转子转速（有符号）
                torque_current = (int16_t)((RxData[4] << 8) | RxData[5]);   // 实际转矩电流（有符号）
                temperature = (int8_t)RxData[6];                            // 电机温度（有符号）
                
                // 直接调用外部定义的处理函数
                Motor_Update_Status(motor_id, rotor_angle, rotor_speed, torque_current, temperature);
            }
        }
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxCallback();
}

/* USER CODE END 1 */