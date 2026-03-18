#ifndef _DBUS_H
#define _DBUS_H

#include "main.h"

typedef signed char int8_t;
typedef signed short int int16_t;
typedef signed int int32_t;
typedef signed long long int64_t;

/* exact-width unsigned integer types */
typedef unsigned char uint8_t;
typedef unsigned short int uint16_t;
typedef unsigned int uint32_t;
typedef unsigned long long uint64_t;
typedef unsigned char bool_t;
typedef float fp32;
typedef double fp64;

 #define DBUS_MAX_LEN   (50)//预定义的数据缓冲区长度
#define DBUS_BUFLEN   (18)//控制帧数据长度
#define DBUS_HUART   huart3//DBUS串口

extern uint8_t dbus_buf[DBUS_BUFLEN];//存储接收到的数据

enum sw_statues
{
    SW_UP,
    SW_MID,
    SW_DOWN,
	  SW_TRIG_UP_MIDDLE,
	  SW_TRIG_MIDDLE_UP,
	  SW_TRIG_MIDDLE_DOWN,
	  SW_TRIG_DOWN_MIDDLE,
	  sw_free,//初始化状态
};

typedef __packed struct
{
  int16_t ch0;
  int16_t ch1;
  int16_t ch2;
  int16_t ch3;
  int16_t roll;
  uint8_t sw1;
  uint8_t sw2;
} rc_info_t;
 
typedef __packed struct
{
  float ch0;
  float ch1;
  float ch2;
  float ch3;
  float roll;
	enum sw_statues sw1;
  enum sw_statues sw2;
} rc_turn_flo;


#define rc_Init   \
{                 \
        0,            \
        0,            \
        0,            \
        0,            \
        0,            \
        sw_free,            \
        sw_free,            \
}



static int uart_receive_dma_no_it(UART_HandleTypeDef* huart, uint8_t* pData, uint32_t Size);
 
void dbus_uart_init(void);//DBUS串口初始化
 static void uart_rx_idle_callback(UART_HandleTypeDef* huart);
void rc_callback_handler(rc_info_t *rc, uint8_t *buff);//遥控器数据处理函数
uint16_t dma_current_data_counter(DMA_Stream_TypeDef *dma_stream);
void uart_receive_handler(UART_HandleTypeDef *huart);

#endif
