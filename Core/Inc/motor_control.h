#ifndef __MOTOR_CONTROL_H
#define __MOTOR_CONTROL_H

#include "main.h"
#include "tim.h"
#include <stdint.h>

/**
 * @brief 电机控制系统的说明 (四轮独立控制)
 * 
 * 电机控制:
 * - 4个电机独立控制 (前LF, 前RF, 后LB, 后RB)
 * - 2个L298N双H桥驱动器（按前后轮分组供电）
 * 
 * 电机布局 (俯视图):
 *   LF ┐     ┌ RF
 *        ┴
 *   LB └     ┘ RB
 * 
 * 电机驱动 (引脚分配):
 * - L298N#1: 前轮供电 (但LF和RF独立控制)
 *   - OUT1/2: 左前轮 (LF) - 引脚: PA0/PA1
 *   - OUT3/4: 右前轮 (RF) - 引脚: PA2/PA3
 *   - ENA: PB6 (TIM4_CH1 PWM) - LF独立速度
 *   - ENB: PB8 (TIM4_CH3 PWM) - RF独立速度
 * 
 * - L298N#2: 后轮供电 (但LB和RB独立控制)
 *   - OUT1/2: 左后轮 (LB) - 引脚: PA4/PA5
 *   - OUT3/4: 右后轮 (RB) - 引脚: PA6/PA7
 *   - ENA: PB7 (TIM4_CH2 PWM) - LB独立速度
 *   - ENB: PB9 (TIM4_CH4 PWM) - RB独立速度
 * 
 * 编码器引脚:
 * - TIM2: 左侧编码器 -> PA15/PB3
 * - TIM3: 右侧编码器 -> PB4/PB5
 *
 * 麦克纳姆轮运动学示例:
 * - 前进: LF+ RF+ LB+ RB+
 * - 后退: LF- RF- LB- RB-
 * - 左平移: LF- RF+ LB+ RB-
 * - 右平移: LF+ RF- LB- RB+
 * - 原地左转: LF- RF+ LB- RB+
 * - 原地右转: LF+ RF- LB+ RB-
 */

// 左前轮 (LF) - L298N#1 OUT1/2 (PA0/PA1)
#define LF_FORWARD() do{ \
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_0,GPIO_PIN_SET); \
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_RESET); \
}while(0)

#define LF_BACKWARD() do{ \
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_0,GPIO_PIN_RESET); \
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_SET); \
}while(0)

#define LF_STOP() do{ \
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_0,GPIO_PIN_RESET); \
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_RESET); \
}while(0)

// 右前轮 (RF) - L298N#1 OUT3/4 (PA2/PA3)
#define RF_FORWARD() do{ \
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,GPIO_PIN_SET); \
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_RESET); \
}while(0)

#define RF_BACKWARD() do{ \
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,GPIO_PIN_RESET); \
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_SET); \
}while(0)

#define RF_STOP() do{ \
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,GPIO_PIN_RESET); \
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_RESET); \
}while(0)

// 左后轮 (LB) - L298N#2 OUT1/2 (PA4/PA5)
#define LB_FORWARD() do{ \
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET); \
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_RESET); \
}while(0)

#define LB_BACKWARD() do{ \
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET); \
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_SET); \
}while(0)

#define LB_STOP() do{ \
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET); \
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_RESET); \
}while(0)

// 右后轮 (RB) - L298N#2 OUT3/4 (PA6/PA7)
#define RB_FORWARD() do{ \
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_SET); \
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_RESET); \
}while(0)

#define RB_BACKWARD() do{ \
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_RESET); \
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_SET); \
}while(0)

#define RB_STOP() do{ \
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_RESET); \
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_RESET); \
}while(0)

#define MAX_PWM 900		//最大占空比
#define MIN_PWM 150		//最小占空比
#define DEAD_ZONE 15 	//摇杆死区

// 编码器和路程测量参数
#define ENCODER_PPR 827.0f      // 编码器每圈脉冲数(根据实际编码器修改)	//827.2
#define WHEEL_DIAMETER 60.0f     // 车轮直径(mm)
#define WHEEL_PERIMETER (3.14159f * WHEEL_DIAMETER)  // 车轮周长(mm)
#define PULSES_PER_MM (ENCODER_PPR / WHEEL_PERIMETER) // 每毫米脉冲数

// 电机初始化函数
void Motor_Init(void);

// 麦克纳姆轮四轮独立控制
void Mecanum_Control(int16_t lf_pwm, int16_t rf_pwm, int16_t lb_pwm, int16_t rb_pwm);
void Car_Stop(void);                  // 停止
void Car_Forward(int16_t speed);      // 前进
void Car_Backward(int16_t speed);     // 后退
void Car_Move_Left(int16_t speed);    // 左平移
void Car_Move_Right(int16_t speed);   // 右平移
void Car_TurnLeft(int16_t speed);     // 原地左转
void Car_TurnRight(int16_t speed);    // 原地右转
void Car_Move_FL(int16_t speed);      // 前左斜移
void Car_Move_FR(int16_t speed);      // 前右斜移
void Car_Move_BL(int16_t speed);      // 后左斜移
void Car_Move_BR(int16_t speed);      // 后右斜移

// 摇杆映射和速度滤波函数
int16_t Map_Joystick_To_PWM(uint8_t joystick_val);
void Speed_Filter_Update(int16_t* left_filtered, int16_t* right_filtered, int16_t left_target, int16_t right_target);

// 编码器函数
int32_t Encoder_GetLeftCount(void);
int32_t Encoder_GetRightCount(void);
void Encoder_ResetCounts(void);

// 里程测量函数
float Odometer_GetLeftDistance(void);   // 获取左侧行驶距离(mm)
float Odometer_GetRightDistance(void);  // 获取右侧行驶距离(mm)
float Odometer_GetTotalDistance(void);  // 获取总行驶距离(mm)
void Odometer_Reset(void);               // 重置里程计

// 速度测量函数
float Speed_GetLeft(void);               // 获取左侧速度(mm/s)
float Speed_GetRight(void);              // 获取右侧速度(mm/s)
void Speed_Update(void);                 // 更新速度测量(定时调用)

// PS2手柄控制函数
void PS2_Control_Car(uint8_t ljoy_lr, uint8_t ljoy_ud, uint8_t btn1);

#endif
