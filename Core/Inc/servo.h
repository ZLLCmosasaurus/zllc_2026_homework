#ifndef __SERVO_H
#define __SERVO_H

#include "stm32f1xx_hal.h"
#include "tim.h"
#include "main.h"
#include "usart.h"

#define SERVO_1_PWM         Servo_1_PWM_Pin
#define SERVO_1_PWM_TIM     &htim1
#define SERVO_1_PWM_CHANNEL TIM_CHANNEL_1

#define SERVO_2_PWM         Servo_2_PWM_Pin
#define SERVO_2_PWM_TIM     &htim1
#define SERVO_2_PWM_CHANNEL TIM_CHANNEL_2

#define SERVO_3_PWM         Servo_3_PWM_Pin
#define SERVO_3_PWM_TIM     &htim1
#define SERVO_3_PWM_CHANNEL TIM_CHANNEL_3

#define SERVO_4_PWM         Servo_4_PWM_Pin
#define SERVO_4_PWM_TIM     &htim1
#define SERVO_4_PWM_CHANNEL TIM_CHANNEL_4

extern Servo_Control_Data servo_data;

void Servo_Init(void);
void SetServoAngle(uint8_t servo, uint8_t angle);
void UpdateServoAngle();

#endif // !__SERVO_H