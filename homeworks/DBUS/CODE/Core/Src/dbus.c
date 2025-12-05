#include "dbus.h"
#include "main.h"

DBUS_Control_Typedef DBUS_Control;

void DBUS_DataHandler(uint8_t *data)
{
    DBUS_Control.Channel._0 = ((int16_t)data[0] | ((int16_t)data[1] << 8)) & 0x07FF;
    DBUS_Control.Channel._1 = (((int16_t)data[1] >> 3) | ((int16_t)data[2] << 5)) & 0x07FF;
    DBUS_Control.Channel._2 = (((int16_t)data[2] >> 6) | ((int16_t)data[3] << 2) |
                               ((int16_t)data[4] << 10)) &
                              0x07FF;
    DBUS_Control.Channel._3      = (((int16_t)data[4] >> 1) | ((int16_t)data[5] << 7)) & 0x07FF;
    DBUS_Control.Switch._1       = ((data[5] >> 4) & 0x000C) >> 2;
    DBUS_Control.Switch._2       = ((data[5] >> 4) & 0x0003);
    DBUS_Control.Mouse.Axis.x    = ((int16_t)data[6]) | ((int16_t)data[7] << 8);
    DBUS_Control.Mouse.Axis.y    = ((int16_t)data[8]) | ((int16_t)data[9] << 8);
    DBUS_Control.Mouse.Axis.z    = ((int16_t)data[10]) | ((int16_t)data[11] << 8);
    DBUS_Control.Mouse.Key.Left  = data[12];
    DBUS_Control.Mouse.Key.Right = data[13];
    DBUS_Control.Mouse.Key.Keys  = ((int16_t)data[14]); // | ((int16_t)data[15] << 8);
}
