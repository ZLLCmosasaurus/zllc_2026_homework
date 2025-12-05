#ifndef __DBUS_H
#define __DBUS_H

#include "main.h"

#define DBUS_CONTROL_DATA_LEN       18

#define DBUS_SWITCH_LOCATION_UP     1
#define DBUS_SWITCH_LOCATION_DOWN   2
#define DBUS_SWITCH_LOCATION_MIDDLE 3

#define DBUS_MOUSE_KEY_RELEASED     0
#define DBUS_MOUSE_KEY_PRESSED      1

#define DBUS_MOUSE_KEY_W            ((uint16_t)1 << 0)
#define DBUS_MOUSE_KEY_S            ((uint16_t)1 << 1)
#define DBUS_MOUSE_KEY_A            ((uint16_t)1 << 2)
#define DBUS_MOUSE_KEY_D            ((uint16_t)1 << 3)
#define DBUS_MOUSE_KEY_Q            ((uint16_t)1 << 4)
#define DBUS_MOUSE_KEY_E            ((uint16_t)1 << 5)
#define DBUS_MOUSE_KEY_SHIFT        ((uint16_t)1 << 6)
#define DBUS_MOUSE_KEY_CTRL         ((uint16_t)1 << 7)

typedef struct {
    struct {
        int16_t _0;
        int16_t _1;
        int16_t _2;
        int16_t _3;
    } Channel;
    struct {
        uint8_t _1;
        uint8_t _2;
    } Switch;
    struct {
        struct {
            int16_t x;
            int16_t y;
            int16_t z;
        } Axis;
        struct {
            uint8_t Left;
            uint8_t Right;
            uint16_t Keys;
        } Key;
    } Mouse;

} DBUS_Control_Typedef;

extern DBUS_Control_Typedef DBUS_Control;

void DBUS_DataHandler(uint8_t *data);

void DBUS_UART_Init();
void DBUS_UART_Handler(UART_HandleTypeDef *huart);

#endif // !__DBUS_H