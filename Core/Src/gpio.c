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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Wheel_1_IN1_Pin|Wheel_1_IN2_Pin|Wheel_2_IN1_Pin|Wheel_2_IN2_Pin
                          |Wheel_3_IN1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Wheel_3_IN2_Pin|PS2_CLK_Pin|Wheel_4_IN1_Pin|Wheel_4_IN2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, PS2_CMD_Pin|PS2_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : Wheel_1_IN1_Pin Wheel_1_IN2_Pin Wheel_2_IN1_Pin Wheel_2_IN2_Pin
                           Wheel_3_IN1_Pin */
  GPIO_InitStruct.Pin = Wheel_1_IN1_Pin|Wheel_1_IN2_Pin|Wheel_2_IN1_Pin|Wheel_2_IN2_Pin
                          |Wheel_3_IN1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : Wheel_2_ENCODER_A_Pin Wheel_2_ENCODER_B_Pin Wheel_3_ENCODER_A_Pin Wheel_3_ENCODER_B_Pin */
  GPIO_InitStruct.Pin = Wheel_2_ENCODER_A_Pin|Wheel_2_ENCODER_B_Pin|Wheel_3_ENCODER_A_Pin|Wheel_3_ENCODER_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : Wheel_3_IN2_Pin Wheel_4_IN1_Pin Wheel_4_IN2_Pin */
  GPIO_InitStruct.Pin = Wheel_3_IN2_Pin|Wheel_4_IN1_Pin|Wheel_4_IN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PS2_DI_Pin */
  GPIO_InitStruct.Pin = PS2_DI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(PS2_DI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PS2_CMD_Pin PS2_CS_Pin PS2_CLK_Pin */
  GPIO_InitStruct.Pin = PS2_CMD_Pin|PS2_CS_Pin|PS2_CLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
