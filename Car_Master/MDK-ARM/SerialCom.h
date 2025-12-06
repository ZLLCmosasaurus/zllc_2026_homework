/**
 * @file SerialCom.h
 * @note 使用说明看这里：
 * @note 0. 确保是USART1，并且开的是接收中断
 * @note 1. 在主函数里调用SerialCom_Init()
 * @note 2. 使用SerialCom_GetEncoder()参数是要赋值的变量地址
 * @note 3. 然后你就自己闭环控制
 */
#ifndef __SERIAL_COM_H
#define __SERIAL_COM_H

#include "main.h"
#include "usart.h"
#include "string.h"


typedef enum Serial_RxStateTypeDef
{
    SERIAL_RX_IDLE,     // 空闲
    SERIAL_RX_HEAD,     // 包头
    SERIAL_RX_BODY,     // 解析有效数据
    SERIAL_RX_END,      // 包尾
} Serial_RxStateTypeDef;


extern uint8_t rx_data_packet[10];
extern uint8_t rx_data;        // 数据字节
extern Serial_RxStateTypeDef SerialCom_RxState;


void SerialCom_Init(void);
uint8_t SerialCom_GetEncoder(int16_t *data1, int16_t *data2, int16_t *data3, int16_t *data4);

#endif

