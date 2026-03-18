/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.h
  * @brief   This file contains all the function prototypes for
  * the can.c file
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CAN_H__
#define __CAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include "motor_control.h"
/* USER CODE END Includes */

extern CAN_HandleTypeDef hcan;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_CAN_Init(void);

/* USER CODE BEGIN Prototypes */
HAL_StatusTypeDef CAN_SendESCCommands(int16_t currents[4]);
HAL_StatusTypeDef CAN_SendMotorCommand(uint8_t motor_id, int16_t current);
HAL_StatusTypeDef CAN_InitFilter(uint8_t motor_id);
void CAN_StartReceive(void);
void CAN_RxCallback(void);

/* 已删除 CAN_SetRxCallback 原型 */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */