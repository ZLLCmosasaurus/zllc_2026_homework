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
#define Wheel_1_IN1_Pin GPIO_PIN_0
#define Wheel_1_IN1_GPIO_Port GPIOA
#define Wheel_1_IN2_Pin GPIO_PIN_1
#define Wheel_1_IN2_GPIO_Port GPIOA
#define Wheel_1_PWM_Pin GPIO_PIN_2
#define Wheel_1_PWM_GPIO_Port GPIOA
#define Wheel_2_PWM_Pin GPIO_PIN_3
#define Wheel_2_PWM_GPIO_Port GPIOA
#define Wheel_2_IN1_Pin GPIO_PIN_4
#define Wheel_2_IN1_GPIO_Port GPIOA
#define Wheel_2_IN2_Pin GPIO_PIN_5
#define Wheel_2_IN2_GPIO_Port GPIOA
#define Wheel_1_ENCODER_A_Pin GPIO_PIN_6
#define Wheel_1_ENCODER_A_GPIO_Port GPIOA
#define Wheel_1_ENCODER_B_Pin GPIO_PIN_7
#define Wheel_1_ENCODER_B_GPIO_Port GPIOA
#define Wheel_2_ENCODER_A_Pin GPIO_PIN_0
#define Wheel_2_ENCODER_A_GPIO_Port GPIOB
#define Wheel_2_ENCODER_A_EXTI_IRQn EXTI0_IRQn
#define Wheel_2_ENCODER_B_Pin GPIO_PIN_1
#define Wheel_2_ENCODER_B_GPIO_Port GPIOB
#define Wheel_2_ENCODER_B_EXTI_IRQn EXTI1_IRQn
#define Wheel_3_IN2_Pin GPIO_PIN_2
#define Wheel_3_IN2_GPIO_Port GPIOB
#define PS2_DI_Pin GPIO_PIN_12
#define PS2_DI_GPIO_Port GPIOB
#define PS2_CMD_Pin GPIO_PIN_13
#define PS2_CMD_GPIO_Port GPIOB
#define PS2_CS_Pin GPIO_PIN_14
#define PS2_CS_GPIO_Port GPIOB
#define PS2_CLK_Pin GPIO_PIN_15
#define PS2_CLK_GPIO_Port GPIOB
#define Servo_1_PWM_Pin GPIO_PIN_8
#define Servo_1_PWM_GPIO_Port GPIOA
#define Servo_2_PWM_Pin GPIO_PIN_9
#define Servo_2_PWM_GPIO_Port GPIOA
#define Servo_3_PWM_Pin GPIO_PIN_10
#define Servo_3_PWM_GPIO_Port GPIOA
#define Servo_4_PWM_Pin GPIO_PIN_11
#define Servo_4_PWM_GPIO_Port GPIOA
#define Wheel_3_IN1_Pin GPIO_PIN_12
#define Wheel_3_IN1_GPIO_Port GPIOA
#define Wheel_3_PWM_Pin GPIO_PIN_15
#define Wheel_3_PWM_GPIO_Port GPIOA
#define Wheel_4_PWM_Pin GPIO_PIN_3
#define Wheel_4_PWM_GPIO_Port GPIOB
#define Wheel_4_IN1_Pin GPIO_PIN_4
#define Wheel_4_IN1_GPIO_Port GPIOB
#define Wheel_4_IN2_Pin GPIO_PIN_5
#define Wheel_4_IN2_GPIO_Port GPIOB
#define Wheel_4_ENCODER_A_Pin GPIO_PIN_6
#define Wheel_4_ENCODER_A_GPIO_Port GPIOB
#define Wheel_4_ENCODER_B_Pin GPIO_PIN_7
#define Wheel_4_ENCODER_B_GPIO_Port GPIOB
#define Wheel_3_ENCODER_A_Pin GPIO_PIN_8
#define Wheel_3_ENCODER_A_GPIO_Port GPIOB
#define Wheel_3_ENCODER_A_EXTI_IRQn EXTI9_5_IRQn
#define Wheel_3_ENCODER_B_Pin GPIO_PIN_9
#define Wheel_3_ENCODER_B_GPIO_Port GPIOB
#define Wheel_3_ENCODER_B_EXTI_IRQn EXTI9_5_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
