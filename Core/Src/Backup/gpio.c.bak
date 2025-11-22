/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
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
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LF_IN1_Pin|LF_IN2_Pin|LB_IN1_Pin|LB_IN2_Pin
                          |RF_IN1_Pin|RF_IN2_Pin|RB_IN1_Pin|RB_IN2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, PS2_CMD_Pin|PS2_CS_Pin|PS2_CLK_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : LF_IN1_Pin LF_IN2_Pin LB_IN1_Pin LB_IN2_Pin
                           RF_IN1_Pin RF_IN2_Pin RB_IN1_Pin RB_IN2_Pin */
  GPIO_InitStruct.Pin = LF_IN1_Pin|LF_IN2_Pin|LB_IN1_Pin|LB_IN2_Pin
                          |RF_IN1_Pin|RF_IN2_Pin|RB_IN1_Pin|RB_IN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PS2_DI_Pin */
  GPIO_InitStruct.Pin = PS2_DI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PS2_DI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PS2_CMD_Pin PS2_CS_Pin PS2_CLK_Pin */
  GPIO_InitStruct.Pin = PS2_CMD_Pin|PS2_CS_Pin|PS2_CLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
