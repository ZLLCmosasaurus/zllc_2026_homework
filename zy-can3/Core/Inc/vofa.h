#ifndef VOFA_H
#define VOFA_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

typedef struct {
    uint8_t buf[128];
    uint16_t head;
    uint16_t tail;
    uint16_t len;
} RingBuffer_TypeDef;

extern RingBuffer_TypeDef vofa_rx_ringbuf;
extern uint8_t vofa_parse_flag;

void VOFA_Init(void);
void VOFA_Send_Data(float angle_set, float angle_fdb, float speed_set, float speed_fdb, float current_out);
void VOFA_Parse_Setpoint(float *angle_set, float *speed_set, 
                         float *kp1, float *ki1, float *kd1, 
                         float *kp2, float *ki2, float *kd2);
void RingBuffer_Put(RingBuffer_TypeDef *rb, uint8_t data);
uint8_t RingBuffer_Get(RingBuffer_TypeDef *rb);

#endif
