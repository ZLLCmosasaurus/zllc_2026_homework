#ifndef __CONTROLLER_H
#define __CONTROLLER_H

#include "stm32f1xx.h"

#define CONTROLLER_DEBUG

void Controller_Init();

void CAN_RxHandler(uint32_t stdId, const uint8_t *rx_buff);
void EXTI_Handler(int16_t GPIO_Pin);

void C620_Motor_Speed_PID_Update(int16_t *speed);
void C620_Motor_Angle_PID_Update(int16_t *angles);

#ifdef CONTROLLER_DEBUG

#define DEBUG_STOP  0x00
#define DEBUG_SPEED 0x01
#define DEBUG_ANGLE 0x02

extern uint8_t debug_status;
extern int16_t speed_gears[3];
extern uint8_t speed_gear;
extern int16_t angle_selections[6];
extern uint8_t angle_select;

void Debug();

#endif

#endif // !__CONTROLLER_H