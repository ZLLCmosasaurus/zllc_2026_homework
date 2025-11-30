#ifndef __WHEEL_H
#define __WHEEL_H

#include "stm32f1xx_hal.h"
#include "tim.h"
#include "main.h"

#define M_PI 3.14159265358979323846

// 左前轮
#define WHEEL_LF             1
#define WHEEL_LF_PWM         Wheel_1_PWM_Pin
#define WHEEL_LF_PWM_TIM     &htim2
#define WHEEL_LF_PWM_CHANNEL TIM_CHANNEL_3
#define WHEEL_LF_IN1_GPIO    Wheel_1_IN1_GPIO_Port
#define WHEEL_LF_IN1         Wheel_1_IN1_Pin
#define WHEEL_LF_IN2_GPIO    Wheel_1_IN2_GPIO_Port
#define WHEEL_LF_IN2         Wheel_1_IN2_Pin

// 右前轮
#define WHEEL_RF             3
#define WHEEL_RF_PWM         Wheel_3_PWM_Pin
#define WHEEL_RF_PWM_TIM     &htim2
#define WHEEL_RF_PWM_CHANNEL TIM_CHANNEL_1
#define WHEEL_RF_IN1_GPIO    Wheel_3_IN1_GPIO_Port
#define WHEEL_RF_IN1         Wheel_3_IN1_Pin
#define WHEEL_RF_IN2_GPIO    Wheel_3_IN2_GPIO_Port
#define WHEEL_RF_IN2         Wheel_3_IN2_Pin

// 左后轮
#define WHEEL_LB             2
#define WHEEL_LB_PWM         Wheel_2_PWM_Pin
#define WHEEL_LB_PWM_TIM     &htim2
#define WHEEL_LB_PWM_CHANNEL TIM_CHANNEL_4
#define WHEEL_LB_IN1_GPIO    Wheel_2_IN1_GPIO_Port
#define WHEEL_LB_IN1         Wheel_2_IN1_Pin
#define WHEEL_LB_IN2_GPIO    Wheel_2_IN2_GPIO_Port
#define WHEEL_LB_IN2         Wheel_2_IN2_Pin

// 右后轮
#define WHEEL_RB             4
#define WHEEL_RB_PWM         Wheel_4_PWM_Pin
#define WHEEL_RB_PWM_TIM     &htim2
#define WHEEL_RB_PWM_CHANNEL TIM_CHANNEL_2
#define WHEEL_RB_IN1_GPIO    Wheel_4_IN1_GPIO_Port
#define WHEEL_RB_IN1         Wheel_4_IN1_Pin
#define WHEEL_RB_IN2_GPIO    Wheel_4_IN2_GPIO_Port
#define WHEEL_RB_IN2         Wheel_4_IN2_Pin

// 控制
#define STOP             0
#define CLOCKWISE        1
#define COUNTERCLOCKWISE 2
#define PLUSE            3

// 物理参数
#define W      0.3f
#define H      0.3f
#define T      (H / 2 + W / 2)
#define RADIUS 0.03f

extern uint64_t WHEEL_PULSE[4];
extern float max_speed;
extern float wheel_target_rpm[4];

void SetWheelTargetRPM(uint8_t wheel, float rpm);
float GetWheelActualRPM(uint8_t wheel);
void UpdateWheelRPM();
void SetWheelDirection(uint8_t wheel, uint8_t direction);
uint8_t GetWheelDirection(uint8_t wheel);
void CarMove(float Vx, float Vy, float Vz);
void CarSpin(uint8_t direction);
void CarGoStraight(float angle);
void Wheel_Init(void);

#endif // !__WHEEL_H