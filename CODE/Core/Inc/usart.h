/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    usart.h
 * @brief   This file contains all the function prototypes for
 *          the usart.c file
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
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern UART_HandleTypeDef huart3;

/* USER CODE BEGIN Private defines */

#define WHEEL_CONTROL        0x11
#define WHEEL_CONTROL_SIZE   12
#define SERVO_CONTROL        0x12
#define SERVO_CONTROL_SIZE   4
#define CAR_STATE            0x13
#define CAR_STATE_SIZE       16
#define CAR_GO_STRAIGHT      0x14
#define CAR_GO_STRAIGHT_SIZE 2
#define CAR_SPIN             0x15
#define CAR_SPIN_SIZE        1
#define CAR_GEAR_SELECT      0x16
#define CAR_GEAR_SELECT_SIZE 1

/* USER CODE END Private defines */

void MX_USART3_UART_Init(void);

/* USER CODE BEGIN Prototypes */
/**
 * @brief 车轮控制参数结构体
 *
 * @param Vx 机器人前后移动速度，前进为正，单位：m/s。
 * @param Vy 机器人左右移动速度，左移为正，单位：m/s。
 * @param Vz 机器人绕 O 点旋转速度，逆时针为正，单位：rad/s
 */
typedef struct {
    float Vx;
    float Vy;
    float Vz;
} Wheel_Control_Data;

/**
 * @brief 舵机控制参数结构体
 *
 * @param servo1 舵机1角度
 * @param servo2 舵机2角度
 * @param servo3 舵机3角度
 * @param servo4 舵机4角度
 */
typedef struct {
    uint8_t servo1;
    uint8_t servo2;
    uint8_t servo3;
    uint8_t servo4;
} Servo_Control_Data;

/**
 * @brief 车向某方向直线运动控制参数结构体
 *
 * @param angle 运动方向的角度
 */
typedef struct {
    int16_t angle;
} Car_Go_Straight_Data;

/**
 * @brief 车向某方向旋转控制参数结构体
 *
 * @param angle 旋转的方向
 */
typedef struct {
    uint8_t direction;
} Car_Spin_Data;

/**
 * @brief 车运动速度挡位选择参数结构体
 *
 * @param gear 车运动挡位，范围为0-3
 */
typedef struct {
    uint8_t gear;
} Car_Gear_Select_Data;

/**
 * @brief 车体状态结构体
 * 包括车的运动状态
 * @param Vx 机器人前后移动速度，前进为正，单位：m/s。
 * @param Vy 机器人左右移动速度，左移为正，单位：m/s。
 * @param Vz 机器人绕 O 点旋转速度，逆时针为正，单位：rad/s
 *
 * @param servo1 舵机1的角度
 * @param servo2 舵机2的角度
 * @param servo3 舵机3的角度
 * @param servo4 舵机4的角度
 */
typedef struct {
    float Vx;
    float Vy;
    float Vz;
    uint8_t servo1;
    uint8_t servo2;
    uint8_t servo3;
    uint8_t servo4;
} Car_State_Data;

/**
 * @brief 校验字节数组并从中解析出车轮控制参数结构体
 *
 * @param control 车轮控制参数结构体
 * @param data 传入的字节数组，长度应等于`WHEEL_CONTROL_SIZE + 2`
 */
void WheelControlDataInit(Wheel_Control_Data *control, uint8_t *data);

/**
 * @brief 校验字节数组并从中解析出车轮控制参数结构体
 *
 * @param control 车轮控制参数结构体
 * @param data 传入的字节数组，长度应等于`SERVO_CONTROL_SIZE + 2`
 */
void ServoControlDataInit(Servo_Control_Data *control, uint8_t *data);

/**
 * @brief 校验字节数组并从中解析出车向某方向直线运动控制参数结构体
 *
 * @param control 车向某方向直线运动控制参数结构体
 * @param data 传入的字节数组，长度应等于`CAR_GO_STRAIGHT_SIZE + 2`
 */
void CarGoStraightDataInit(Car_Go_Straight_Data *control, uint8_t *data);

/**
 * @brief 校验字节数组并从中解析出车向某方向直线运动控制参数结构体
 *
 * @param control 车向某方向旋转控制参数结构体
 * @param data 传入的字节数组，长度应等于`CAR_SPIN_SIZE + 2`
 */
void CarSpinDataInit(Car_Spin_Data *control, uint8_t *data);

/**
 * @brief 校验字节数组并从中解析出车运动速度挡位选择参数结构体
 *
 * @param control 车运动速度挡位选择参数结构体
 * @param data 传入的字节数组，长度应等于`CAR_GEAR_SELECT_SIZE + 2`
 */
void CarGearSelectDataInit(Car_Gear_Select_Data *control, uint8_t *data);

/**
 * @brief 将结构体中的数据存入字节数组中
 *
 * @param control 车体状态结构体
 * @param data 要存入的字节数组，长度应等于`CAR_STATE_SIZE + 2`
 */
void CarStateDataInit(Car_State_Data *control, uint8_t *data);

/**
 * @brief 实现大小端数据转换
 *
 * @param bytes 要转换的字节数组
 * @param length 要转换的长度
 */
uint8_t *EndianTransfer(uint8_t *bytes, size_t length);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

