#include "vofa.h"
#include "usart.h" 
#include <stdio.h>
#include <string.h>

RingBuffer_TypeDef vofa_rx_ringbuf = {0};
uint8_t vofa_parse_flag = 0;

void VOFA_Init(void) {
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
}

void RingBuffer_Put(RingBuffer_TypeDef *rb, uint8_t data) {
    if (rb->len < sizeof(rb->buf)) {
        rb->buf[rb->head] = data;
        rb->head = (rb->head + 1) % sizeof(rb->buf);
        rb->len++;
    }
}

uint8_t RingBuffer_Get(RingBuffer_TypeDef *rb) {
    uint8_t data = 0;
    if (rb->len > 0) {
        data = rb->buf[rb->tail];
        rb->tail = (rb->tail + 1) % sizeof(rb->buf);
        rb->len--;
    }
    return data;
}

void VOFA_Send_Data(float angle_set, float angle_fdb, float speed_set, float speed_fdb, float current_out) {
    char buf[128];
   
    sprintf(buf, "samples:%.2f,%.2f,%.2f,%.2f,%.2f\n", 
            angle_set, angle_fdb, speed_set, speed_fdb, current_out);
    
    __HAL_UART_DISABLE_IT(&huart1, UART_IT_RXNE);

    HAL_UART_Transmit(&huart1, (uint8_t*)buf, strlen(buf), 100);
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
}

void VOFA_Parse_Setpoint(float *angle_set, float *speed_set, 
                         float *kp1, float *ki1, float *kd1, 
                         float *kp2, float *ki2, float *kd2) {
    if (vofa_parse_flag) {
        char parse_buf[128];
        uint16_t i = 0;
        uint8_t ch;

        while (vofa_rx_ringbuf.len > 0 && i < sizeof(parse_buf)-1) {
            ch = RingBuffer_Get(&vofa_rx_ringbuf);
            parse_buf[i++] = ch;
            if (ch == '\n') break;
        }
        parse_buf[i] = '\0';

        if (sscanf(parse_buf, "%f,%f,%f,%f,%f,%f,%f,%f", 
                   angle_set, speed_set, kp1, ki1, kd1, kp2, ki2, kd2) == 8) {

        }
        
        vofa_parse_flag = 0;
    }
}

