/**
           ____                    _____ _______ _____       XTARK@塔克创新
          / __ \                  / ____|__   __|  __ \
         | |  | |_ __   ___ _ __ | |       | |  | |__) |
         | |  | | '_ \ / _ \ '_ \| |       | |  |  _  /
         | |__| | |_) |  __/ | | | |____   | |  | | \ \
          \____/| .__/ \___|_| |_|\_____|  |_|  |_|  \_\
                | |
                |_|                OpenCTR   机器人控制器

  ******************************************************************************
  *
  * 版权所有： XTARK@塔克创新  版权所有，盗版必究
  * 公司网站： www.xtark.cn   www.tarkbot.com
  * 淘宝店铺： https://xtark.taobao.com
  * 塔克微信： 塔克创新（关注公众号，获取最新更新资讯）
  *
  ******************************************************************************
  * @作  者  塔克创新团队
  * @内  容  PS2无线手柄函数文件
  *
  ******************************************************************************
  * @说  明
  *
  *   PS2数据定义
  *   BYTE   DATA   解释
  *   01     idle
  *   02     0x73   手柄工作模式
  *   03     0x5A   Bit0  Bit1  Bit2  Bit3  Bit4  Bit5  Bit6  Bit7
  *   04     data   SLCT  JOYR  JOYL  STRT   UP   RGIHT  DOWN   L
  *   05     data   L2     R2     L1    R1   Y     B     A      X
  *   06     data   右边摇杆  0x00 = 左    0xff = 右    0x80 = 中间
  *   07     data   右边摇杆  0x00 = 上    0xff = 下    0x7f = 中间
  *   08     data   左边摇杆  0x00 = 左    0xff = 右    0x80 = 中间
  *   09     data   左边摇杆  0x00 = 上    0xff = 下    0x7f = 中间
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __AX_PS2_H
#define __AX_PS2_H

/* Includes ------------------------------------------------------------------*/

#include "stm32f1xx_hal.h"
// PS2手柄键值数据结构体
typedef struct
{
    uint8_t mode; /* 手柄的工作模式 */

    uint8_t btn1; /* B0:SLCT B1:JL  B2:JR B3:STRT B4:UP B5:R B6:DOWN  B7:L   */

    uint8_t btn2; /* B0:L2   B1:R2  B2:L1 B3:R1   B4:Y  B5:B B6:A     B7:X */

    uint8_t RJoy_LR; /* 右边摇杆  0x00 = 左    0xff = 右   */

    uint8_t RJoy_UD; /* 右边摇杆  0x00 = 上    0xff = 下   */

    uint8_t LJoy_LR; /* 左边摇杆  0x00 = 左    0xff = 右   */

    uint8_t LJoy_UD; /* 左边摇杆  0x00 = 上    0xff = 下   */

} JOYSTICK_TypeDef;

#define BUTTON_NO_CHANGE      0x00
#define BUTTON_SELECT         0x01
#define BUTTON_LS             0x02
#define BUTTON_RS             0x03
#define BUTTON_START          0x04
#define BUTTON_UP             0x05
#define BUTTON_RIGHT          0x06
#define BUTTON_DOWN           0x07
#define BUTTON_LEFT           0x08
#define BUTTON_L2             0x09
#define BUTTON_R2             0x0A
#define BUTTON_L1             0x0B
#define BUTTON_R1             0x0C
#define BUTTON_Y              0x0D
#define BUTTON_B              0x0E
#define BUTTON_A              0x0F
#define BUTTON_X              0x10

#define BUTTON_STATE_RELEASED 0x00
#define BUTTON_STATE_PRESSED  0x01

extern JOYSTICK_TypeDef JoyStick;
extern uint8_t JoyStickControl;

/*** PS2无线手柄操作函数 **********/
void AX_PS2_Init(void);    // PS2初始化
uint16_t AX_PS2_ScanKey(); // PS2获取按键及摇杆数值
void KeyEventHandler(uint8_t key, uint8_t state);

#endif

/******************* (C) 版权 2023 XTARK **************************************/
