/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
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
#include "can.h"

/* USER CODE BEGIN 0 */
CAN_TxHeaderTypeDef txcan;
CAN_RxHeaderTypeDef rxcan;
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
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_6TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
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
  //配置过滤器（当前配置过滤器为接收所有报文，不筛选）
  CAN_FilterTypeDef can_filter_config;
  //16位屏蔽模式
  can_filter_config.FilterMode=CAN_FILTERMODE_IDMASK;
  can_filter_config.FilterScale=CAN_FILTERSCALE_16BIT;
  //掩码
  can_filter_config.FilterMaskIdHigh=0x7FF<<5;
  can_filter_config.FilterMaskIdLow=0;
  //ID
  can_filter_config.FilterIdHigh=0x201<<5;
  can_filter_config.FilterIdLow=0;
  //使用过滤器组0，发送到FIFO0
  can_filter_config.FilterBank=0;
  can_filter_config.FilterFIFOAssignment=CAN_FILTER_FIFO0;
  //激活过滤器，从CAN过滤器默认14
  can_filter_config.FilterActivation=CAN_FILTER_ENABLE;
  can_filter_config.SlaveStartFilterBank=14;

  HAL_CAN_ConfigFilter(&hcan,&can_filter_config);

  //启动CAN
  HAL_CAN_Start(&hcan);
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
    HAL_NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
    HAL_NVIC_SetPriority(CAN1_RX1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX1_IRQn);
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
    HAL_NVIC_DisableIRQ(USB_LP_CAN1_RX0_IRQn);
    HAL_NVIC_DisableIRQ(CAN1_RX1_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
//发送标准ID的数据帧
  void My_CAN_Send(uint16_t id,uint8_t* data,uint8_t data_len)
  {
    uint32_t mail=CAN_TX_MAILBOX0;

    txcan.DLC=data_len;
    txcan.StdId=id;
    txcan.IDE=CAN_ID_STD;
    txcan.RTR=CAN_RTR_DATA;

    HAL_CAN_AddTxMessage(&hcan,&txcan,data,&mail);

    //等待发送完成
    while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan)!=3);
  }
  void My_CAN_Sete(int16_t e)
  {
    uint8_t tx_data[8]={0};
    tx_data[0]=e>>8;
    tx_data[1]=e&0xff;
    My_CAN_Send(0x200,tx_data,8);
  }
//请先定义 uint8_t data_receive[8];
//返回收到数据的长度，并把数据存在这个数组里
  uint8_t My_CAN_Receive(uint8_t* data_receive)
  {
    if(HAL_CAN_GetRxFifoFillLevel(&hcan,CAN_FILTER_FIFO0)==0)
    {
      return 0;
    }
    HAL_CAN_GetRxMessage(&hcan,CAN_FILTER_FIFO0,&rxcan,data_receive);
    return rxcan.DLC;
  }
/* USER CODE END 1 */
