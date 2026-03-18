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
#define M1_IN2_Pin GPIO_PIN_14
#define M1_IN2_GPIO_Port GPIOC
#define M1_IN1_Pin GPIO_PIN_15
#define M1_IN1_GPIO_Port GPIOC
#define M2_IN1_Pin GPIO_PIN_0
#define M2_IN1_GPIO_Port GPIOA
#define M2_IN2_Pin GPIO_PIN_1
#define M2_IN2_GPIO_Port GPIOA
#define E1_A_Pin GPIO_PIN_4
#define E1_A_GPIO_Port GPIOA
#define E1_A_EXTI_IRQn EXTI4_IRQn
#define E1_B_Pin GPIO_PIN_5
#define E1_B_GPIO_Port GPIOA
#define E1_B_EXTI_IRQn EXTI9_5_IRQn
#define E2_A_Pin GPIO_PIN_6
#define E2_A_GPIO_Port GPIOA
#define E2_A_EXTI_IRQn EXTI9_5_IRQn
#define SERVO2_Pin GPIO_PIN_7
#define SERVO2_GPIO_Port GPIOA
#define SERVO3_Pin GPIO_PIN_0
#define SERVO3_GPIO_Port GPIOB
#define E2_B_Pin GPIO_PIN_1
#define E2_B_GPIO_Port GPIOB
#define E2_B_EXTI_IRQn EXTI1_IRQn
#define M3_IN1_Pin GPIO_PIN_10
#define M3_IN1_GPIO_Port GPIOB
#define M3_IN2_Pin GPIO_PIN_11
#define M3_IN2_GPIO_Port GPIOB
#define E3_A_Pin GPIO_PIN_13
#define E3_A_GPIO_Port GPIOB
#define E3_A_EXTI_IRQn EXTI15_10_IRQn
#define E3_B_Pin GPIO_PIN_14
#define E3_B_GPIO_Port GPIOB
#define E3_B_EXTI_IRQn EXTI15_10_IRQn
#define E4_A_Pin GPIO_PIN_15
#define E4_A_GPIO_Port GPIOB
#define E4_A_EXTI_IRQn EXTI15_10_IRQn
#define M1_PWM_Pin GPIO_PIN_8
#define M1_PWM_GPIO_Port GPIOA
#define M2_PWM_Pin GPIO_PIN_9
#define M2_PWM_GPIO_Port GPIOA
#define M3_PWM_Pin GPIO_PIN_10
#define M3_PWM_GPIO_Port GPIOA
#define M4_PWM_Pin GPIO_PIN_11
#define M4_PWM_GPIO_Port GPIOA
#define E4_B_Pin GPIO_PIN_12
#define E4_B_GPIO_Port GPIOA
#define E4_B_EXTI_IRQn EXTI15_10_IRQn
#define M4_IN1_Pin GPIO_PIN_3
#define M4_IN1_GPIO_Port GPIOB
#define M4_IN2_Pin GPIO_PIN_4
#define M4_IN2_GPIO_Port GPIOB
#define SERVO1_Pin GPIO_PIN_8
#define SERVO1_GPIO_Port GPIOB
#define SERVO4_Pin GPIO_PIN_9
#define SERVO4_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
