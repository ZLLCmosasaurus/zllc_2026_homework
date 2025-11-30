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

// 串级PID控制器结构体
typedef struct {
    pid_controller_t angle_pid;   // 角度环PID控制器
    pid_controller_t speed_pid;   // 速度环PID控制器
    float target_angle;           // 目标角度
    float target_speed;           // 目标速度
    float current_angle;          // 当前角度
    float current_speed;          // 当前速度
} cascade_pid_controller_t;

// 电机连续角度跟踪变量（用于多圈角度计算）
extern uint8_t  motor_angle_inited[4];
extern uint16_t motor_last_angle_raw[4];
extern float    motor_continuous_angle[4];

// UART发送状态标志
extern volatile uint8_t uart_tx_busy;

/* USER CODE END Private defines */

void MX_CAN_Init(void);

/* USER CODE BEGIN Prototypes */
void can_dji_motor_control(uint8_t motor_id, int16_t current);
void can_dji_motor_control_multi(int16_t current1, int16_t current2, int16_t current3, int16_t current4);
void can_receive_callback(CAN_RxHeaderTypeDef *rx_header, uint8_t rx_data[]);
void pid_init(pid_controller_t *pid, float kp, float ki, float kd);
float pid_calculate(pid_controller_t *pid, float target, float current, float dt);
void cascade_pid_init(cascade_pid_controller_t *cascade_pid, float angle_kp, float angle_ki, float angle_kd, float speed_kp, float speed_ki, float speed_kd);
float cascade_pid_calculate(cascade_pid_controller_t *cascade_pid, float target_angle, float current_angle, float current_speed, float dt);
void motor_rotate_to_angle(uint8_t motor_id, float target_angle, float speed_limit);

// 电机反馈数据外部声明
extern motor_feedback_t motor_feedback[4];
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */

