/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.h
  * @brief   This file contains all the function prototypes for
  *          the can.c file
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
#ifndef __CAN_H__
#define __CAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern CAN_HandleTypeDef hcan;

/* USER CODE BEGIN Private defines */
#define DJI_3508_CAN_ID 0x200  // DJI 3508电机的CAN ID
#define DJI_3508_FEEDBACK_ID_START 0x201  // DJI 3508电机反馈ID起始地址

// 电机反馈数据结构
typedef struct {
    int16_t angle;      // 电机角度
    int16_t speed;      // 电机转速
    int16_t current;    // 电机电流
    int16_t temp;       // 电机温度
} motor_feedback_t;

// PID控制器结构体
typedef struct {
    float kp;           // 比例系数
    float ki;           // 积分系数
    float kd;           // 微分系数
    float target;       // 目标值
    float current;      // 当前值
    float error;        // 当前误差
    float last_error;   // 上次误差
    float integral;     // 积分项
    float output;       // 输出值
} pid_controller_t;

/* USER CODE END Private defines */

void MX_CAN_Init(void);

/* USER CODE BEGIN Prototypes */
void can_dji_motor_control(uint8_t motor_id, int16_t current);
void can_dji_motor_control_multi(int16_t current1, int16_t current2, int16_t current3, int16_t current4);
void can_receive_callback(CAN_RxHeaderTypeDef *rx_header, uint8_t rx_data[]);
void pid_init(pid_controller_t *pid, float kp, float ki, float kd);
float pid_calculate(pid_controller_t *pid, float target, float current);

// 电机反馈数据外部声明
extern motor_feedback_t motor_feedback[4];
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */

