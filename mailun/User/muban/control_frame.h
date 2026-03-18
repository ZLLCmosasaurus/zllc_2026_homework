#ifndef __CONTROL_FRAME_H
#define __CONTROL_FRAME_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

typedef struct
{
  uint16_t ch0;
  uint16_t ch1;
  uint16_t ch2;
  uint16_t ch3;

  uint8_t  s1;
  uint8_t  s2;

  int16_t  mouse_x;
  int16_t  mouse_y;
  int16_t  mouse_z;

  uint8_t  mouse_left;
  uint8_t  mouse_right;

  uint16_t key;
  uint16_t reserve;
} ControlFrame_t;

void ControlFrame_Init(ControlFrame_t *f);

uint16_t Control_Map_Channel_FromFloat(float x);
uint16_t Control_Map_Channel_FromADC(uint16_t adc, uint16_t adc_min, uint16_t adc_max);
uint16_t Control_Map_Channel_FromVoltage(float v);
int16_t  Control_Map_Mouse_FromFloat(float x);

void ControlFrame_Pack(uint8_t buf[18], const ControlFrame_t *f);
void ControlFrame_Send(UART_HandleTypeDef *huart, ControlFrame_t *f, uint8_t buf[18]);
void ControlFrame_Decode(const uint8_t buf[18], ControlFrame_t *f);
HAL_StatusTypeDef ControlFrame_ReceiveAndDecode(UART_HandleTypeDef *huart,
                                                ControlFrame_t *f,
                                                uint8_t buf[18],
                                                uint32_t timeout);

#ifdef __cplusplus
}
#endif

#endif /* __CONTROL_FRAME_H */
