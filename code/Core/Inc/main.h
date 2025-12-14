/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LF_IN1_Pin GPIO_PIN_0
#define LF_IN1_GPIO_Port GPIOA
#define LF_IN2_Pin GPIO_PIN_1
#define LF_IN2_GPIO_Port GPIOA
#define LB_IN1_Pin GPIO_PIN_2
#define LB_IN1_GPIO_Port GPIOA
#define LB_IN2_Pin GPIO_PIN_3
#define LB_IN2_GPIO_Port GPIOA
#define RF_IN1_Pin GPIO_PIN_4
#define RF_IN1_GPIO_Port GPIOA
#define RF_IN2_Pin GPIO_PIN_5
#define RF_IN2_GPIO_Port GPIOA
#define RB_IN1_Pin GPIO_PIN_6
#define RB_IN1_GPIO_Port GPIOA
#define RB_IN2_Pin GPIO_PIN_7
#define RB_IN2_GPIO_Port GPIOA
#define UART_TX_Pin GPIO_PIN_10
#define UART_TX_GPIO_Port GPIOB
#define UART_RX_Pin GPIO_PIN_11
#define UART_RX_GPIO_Port GPIOB
#define PS2_DI_Pin GPIO_PIN_12
#define PS2_DI_GPIO_Port GPIOB
#define PS2_CMD_Pin GPIO_PIN_13
#define PS2_CMD_GPIO_Port GPIOB
#define PS2_CS_Pin GPIO_PIN_14
#define PS2_CS_GPIO_Port GPIOB
#define PS2_CLK_Pin GPIO_PIN_15
#define PS2_CLK_GPIO_Port GPIOB
#define ENC_L_A_Pin GPIO_PIN_15
#define ENC_L_A_GPIO_Port GPIOA
#define ENC_L_B_Pin GPIO_PIN_3
#define ENC_L_B_GPIO_Port GPIOB
#define ENC_R_A_Pin GPIO_PIN_4
#define ENC_R_A_GPIO_Port GPIOB
#define ENC_R_B_Pin GPIO_PIN_5
#define ENC_R_B_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
