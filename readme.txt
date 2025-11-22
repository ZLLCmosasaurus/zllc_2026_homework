PS2手柄demo - STM32F103C8T6

【项目说明】
 * 软件名称：   PS2无线手柄例程(HAL库工程版)
 * 主要功能：   获取手柄键值

【引脚及时钟配置】
 * PB12 - PB15 配置用于与PS2手柄进行通信(MISO, MOSI, CS, CLK)，具体配置见ioc文件
 * PA9 PA10 配置用于串口通信，可以根据需求在ioc中更改引脚配置
 * 使用外部高速时钟，时钟频率72MHz

【使用方法说明】
 * ax_ps2.h中提供了手柄初始化函数 AX_PS2_Init()，用于对PS2手柄通信进行初始化
 * (重要)手柄初始化之后，提供了用于与手柄发起通信并读取手柄按键值的函数 AX_PS2_ScanKey(JOYSTICK_TypeDef *JoystickStruct)，使用该函数时传入一个JOYSTICK_TypeDef类型的变量指针用于存放获取的手柄数据
 * ax_ps2.h中内置了一个较高精度的微秒级延时函数delay_us(us)，可以根据需求使用
 * 配置了一个UART通信通道，可以用于串口通信，并将printf重定向用于在串口打印信息，可根据需求在ioc文件和代码中关闭或配置用于其他用途

 JOYSTICK_TypeDef结构体说明：
 typedef struct			 				
{
  uint8_t mode;		    /* 手柄的工作模式 */

  uint8_t btn1;         
  /*
    uint8_t 是一个8位的数据，用于存储一组按钮的状态，每一位代表一个按钮，详情如下：
    B0:SLCT B1:JR  B0:JL B3:STRT B4:UP B5:R B6:DOWN  B7:L   
   */

  uint8_t btn2;         /* 说明同上 B0:L2   B1:R2  B2:L1 B3:R1   B4:Y  B5:B B6:A     B7:X */

  uint8_t RJoy_LR;      /* 右摇杆左右  0x00 = 左    0xff = 右   */

  uint8_t RJoy_UD;      /* 右摇杆上下  0x00 = 上    0xff = 下   */

  uint8_t LJoy_LR;      /* 左摇杆左右  0x00 = 左    0xff = 右   */

  uint8_t LJoy_UD;      /* 左摇杆上下  0x00 = 上    0xff = 下   */
	
}JOYSTICK_TypeDef;