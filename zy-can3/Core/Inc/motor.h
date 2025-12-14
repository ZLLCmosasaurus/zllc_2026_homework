#ifndef MOTOR_H
#define MOTOR_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

typedef struct {
    int16_t angle_raw;    
    float angle;          
    int16_t speed_raw;  
    float speed;         
    int16_t current_raw;  
    float current;        
    uint8_t temp;       
} Motor_State_TypeDef;

extern Motor_State_TypeDef motor_state[4];  // 4个电机状态
extern uint8_t can_rx_flag;                 // CAN接收标志

void CAN_Motor_Init(void);
void CAN_Send_Motor_Cmd(uint8_t motor_id, float current);
void CAN_Receive_Init(void);

#endif

