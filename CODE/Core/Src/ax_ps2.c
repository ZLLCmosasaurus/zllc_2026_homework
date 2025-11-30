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
  *   06     data   右边摇杆  0x00 = 左    0xff = 右
  *   07     data   右边摇杆  0x00 = 上    0xff = 下
  *   08     data   左边摇杆  0x00 = 左    0xff = 右
  *   09     data   左边摇杆  0x00 = 上    0xff = 下
  *
  ******************************************************************************
  */

#include "ax_ps2.h"
#include "usart.h"
#include "servo.h"
#include "wheel.h"

// PS2手柄的输入输出口
#define DI()    HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) // 数据输入引脚

#define CMD_H() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET)   // 命令位高
#define CMD_L() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET) // 命令位低

#define CS_H()  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET)   // CS拉高(别名ATT)
#define CS_L()  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET) // CS拉低(别名ATT)

#define CLK_H() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET)   // 时钟拉高
#define CLK_L() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET) // 时钟拉低

const uint8_t PS2_cmnd[9]  = {0x01, 0x42, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // 请求获取数据命令
static uint8_t PS2_data[9] = {0};                                                    // 接收的数据
JOYSTICK_TypeDef JoyStick;
uint8_t JoyStickControl = 0;

static const uint8_t BUTTON_MAP[2][8] = {
    // btn1 (PS2_data[3]) 的按键映射
    {BUTTON_LEFT, BUTTON_DOWN, BUTTON_RIGHT, BUTTON_UP,
     BUTTON_START, BUTTON_RS, BUTTON_LS, BUTTON_SELECT},
    // btn2 (PS2_data[4]) 的按键映射
    {BUTTON_X, BUTTON_A, BUTTON_B, BUTTON_Y,
     BUTTON_R1, BUTTON_L1, BUTTON_R2, BUTTON_L2}};

void DWT_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void delay_us(uint16_t us)
{
    uint32_t ticks = us * (SystemCoreClock / 1000000);
    uint32_t told  = DWT->CYCCNT;
    uint32_t tnow;

    while (1) {
        tnow = DWT->CYCCNT;
        if ((tnow - told) >= ticks) break;
    }
}

/**
 * @简  述  PS2初始化
 * @参  数  无
 * @返回值  无
 */
void AX_PS2_Init(void)
{
    // 结构体初始化
    JoyStick.btn1 = 0x00;
    JoyStick.btn2 = 0x00;

    DWT_Init();
    // 关闭PS2手柄使能
    CS_H();
}

/**
 * @简  述  PS2数据读写函数
 * @参  数  cmd:要写入的命令
 * @返回值  读出数据
 */
static uint8_t PS2_ReadWriteData(uint8_t cmd)
{
    volatile uint8_t res = 0;
    volatile uint8_t ref;

    // 写入命令，并读取一个1字节数据
    for (ref = 0x01; ref > 0x00; ref <<= 1) {
        ////输出一位数据
        if (ref & cmd)
            CMD_H();
        else
            CMD_L();

        CLK_L();
        delay_us(16);

        // 读取一位数据
        if (DI())
            res |= ref;
        CLK_H();
        delay_us(16);
    }

    // 返回读出数据
    return res;
}

/**
 * @简  述  PS2获取按键及摇杆数值。
 * @参  数  *JoyStick 手柄键值结构体
 * @返回值  无
 */
uint16_t AX_PS2_ScanKey()
{
    uint16_t button_and_state = 0x0000;
    uint8_t i;

    // 使能手柄
    CS_L();

    // 读取PS2数据
    for (i = 0; i < 9; i++) {
        PS2_data[i] = PS2_ReadWriteData(PS2_cmnd[i]);
    }

    // 关闭使能
    CS_H();

    // 检测按键变化
    if ((JoyStick.btn1 != ~PS2_data[3]) || (JoyStick.btn2 != ~PS2_data[4])) {
        uint8_t btn_changes[2] = {
            JoyStick.btn1 ^ ~PS2_data[3], // 变化的位
            JoyStick.btn2 ^ ~PS2_data[4]};

        uint8_t current_states[2] = {
            ~PS2_data[3],
            ~PS2_data[4]};

        // 检查两个字节的按键变化
        for (int byte_idx = 0; byte_idx < 2; byte_idx++) {
            uint8_t changes = btn_changes[byte_idx];
            uint8_t current = current_states[byte_idx];

            // 检查每个位的变化
            for (int bit_idx = 7; bit_idx >= 0; bit_idx--) {
                if (changes & (1 << bit_idx)) {
                    uint8_t button_code = BUTTON_MAP[byte_idx][7 - bit_idx];
                    if (button_code != BUTTON_NO_CHANGE) {
                        uint8_t state    = (current & (1 << bit_idx)) ? BUTTON_STATE_PRESSED : BUTTON_STATE_RELEASED;
                        button_and_state = (button_code << 8) | state;
                        break; // 只处理第一个变化，或者去掉break处理所有变化
                    }
                }
            }
        }
    }

    // 数值传递
    JoyStick.mode    = PS2_data[1];
    JoyStick.btn1    = ~PS2_data[3];
    JoyStick.btn2    = ~PS2_data[4];
    JoyStick.RJoy_LR = PS2_data[5];
    JoyStick.RJoy_UD = PS2_data[6];
    JoyStick.LJoy_LR = PS2_data[7];
    JoyStick.LJoy_UD = PS2_data[8];

    return button_and_state;
}

/******************* (C) 版权 2023 XTARK **************************************/

void KeyEventHandler(uint8_t key, uint8_t state)
{
    static uint8_t last_key   = 0;
    static uint8_t last_state = 0;
    if (key != BUTTON_NO_CHANGE) {
        if (state == BUTTON_STATE_RELEASED) {
            static uint8_t gear      = 1;
            static float car_gear[4] = {0.0f, 0.5f, 1.0f, 1.5f};

            if (key == BUTTON_L1) {
                gear      = (gear == 0) ? 0 : (gear - 1);
                max_speed = car_gear[gear];
            }
            if (key == BUTTON_R1) {
                gear      = (gear == 3) ? 3 : (gear + 1);
                max_speed = car_gear[gear];
            }
        }
        last_key   = key;
        last_state = state;
    } else if (key == BUTTON_NO_CHANGE) {
        if (last_state == BUTTON_STATE_RELEASED) {
            if (last_key == BUTTON_A) {
                servo_data.servo4 += 1;
                SetServoAngle(4, servo_data.servo4);
            }
            if (last_key == BUTTON_B) {
                if (servo_data.servo4 != 0) {
                    servo_data.servo4 -= 1;
                }
                SetServoAngle(4, servo_data.servo4);
            }
            if (last_key == BUTTON_X) {
                servo_data.servo2 += 1;
                SetServoAngle(2, servo_data.servo2);
            }
            if (last_key == BUTTON_Y) {
                if (servo_data.servo2 != 0) {
                    servo_data.servo2 -= 1;
                }
                SetServoAngle(2, servo_data.servo2);
            }
            if (last_key == BUTTON_UP) {
                servo_data.servo3 += 1;
                SetServoAngle(3, servo_data.servo3);
            }
            if (last_key == BUTTON_DOWN) {
                if (servo_data.servo3 != 0) {
                    servo_data.servo3 -= 1;
                }
                SetServoAngle(3, servo_data.servo3);
            }
            if (last_key == BUTTON_L2) {
                servo_data.servo1 += 1;
                SetServoAngle(1, servo_data.servo1);
            }
            if (last_key == BUTTON_R2) {
                if (servo_data.servo1 != 0) {
                    servo_data.servo1 -= 1;
                }
                SetServoAngle(1, servo_data.servo1);
            }
        }
    }
}