#include "dbus.h"
#include "main.h"
#include "string.h"

uint8_t dbus_rx_buffer[50];
uint8_t dbus_frame_data[18];
DBUS_Control_Typedef DBUS_Control;

void DBUS_DataHandler(uint8_t *data)
{
    DBUS_Control.Channel._0 = ((int16_t)data[0] | ((int16_t)data[1] << 8)) & 0x07FF;
    DBUS_Control.Channel._1 = (((int16_t)data[1] >> 3) | ((int16_t)data[2] << 5)) & 0x07FF;
    DBUS_Control.Channel._2 = (((int16_t)data[2] >> 6) | ((int16_t)data[3] << 2) |
                               ((int16_t)data[4] << 10)) &
                              0x07FF;
    DBUS_Control.Channel._3 = (((int16_t)data[4] >> 1) | ((int16_t)data[5] << 7)) & 0x07FF;
    DBUS_Control.Switch._1 = ((data[5] >> 4) & 0x000C) >> 2;
    DBUS_Control.Switch._2 = ((data[5] >> 4) & 0x0003);
    DBUS_Control.Mouse.Axis.x = ((int16_t)data[6]) | ((int16_t)data[7] << 8);
    DBUS_Control.Mouse.Axis.y = ((int16_t)data[8]) | ((int16_t)data[9] << 8);
    DBUS_Control.Mouse.Axis.z = ((int16_t)data[10]) | ((int16_t)data[11] << 8);
    DBUS_Control.Mouse.Key.Left = data[12];
    DBUS_Control.Mouse.Key.Right = data[13];
    DBUS_Control.Mouse.Key.Keys = ((int16_t)data[14]); // | ((int16_t)data[15] << 8);
}

void DBUS_UART_Handler(UART_HandleTypeDef *huart)
{
    __HAL_UART_CLEAR_IDLEFLAG(huart);
    __HAL_DMA_DISABLE(huart->hdmarx);

    uint16_t received_count = sizeof(dbus_rx_buffer) - __HAL_DMA_GET_COUNTER(huart->hdmarx);

    if (received_count == 18)
    {
        memcpy(dbus_frame_data, dbus_rx_buffer, 18);
        DBUS_DataHandler(dbus_frame_data);
    }

    __HAL_DMA_SET_COUNTER(huart->hdmarx, sizeof(dbus_rx_buffer));
    __HAL_DMA_ENABLE(huart->hdmarx);
}

void DBUS_UART_Init(UART_HandleTypeDef *huart)
{
    __HAL_UART_CLEAR_IDLEFLAG(huart);
    __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
    HAL_UART_Receive_DMA(huart, dbus_rx_buffer, sizeof(dbus_rx_buffer));
}