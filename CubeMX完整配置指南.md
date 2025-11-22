# STM32CubeMX 完整配置指南 - 四电机+四舵机混合控制系统

## 📌 项目基本信息

- **MCU型号**：STM32F103C8T6
- **封装**：LQFP48
- **Flash**：64KB
- **RAM**：20KB
- **主频**：72MHz
- **电机配置**：4个麦克纳姆轮电机 + 2个L298N电驱
- **舵机配置**：4个MG996R舵机
- **驱动方式**：
  - 电机：每个L298N控制2个同侧电机（左前+左后 / 右前+右后）
  - 舵机：独立PWM控制，50Hz频率

---

## 1. 新建项目

### 1.1 选择MCU
1. 打开STM32CubeMX
2. **File → New Project**
3. 在MCU Selector中搜索：`STM32F103C8`
4. 选择 **STM32F103C8Tx**
5. 点击 **Start Project**

---

## 2. RCC时钟配置

### 2.1 RCC基本设置
**路径**：`Pinout & Configuration → System Core → RCC`

| 参数 | 设置值 |
|------|--------|
| **HSE (High Speed External)** | Crystal/Ceramic Resonator |
| **LSE (Low Speed External)** | Disable |

**说明**：使用外部8MHz晶振

### 2.2 时钟树配置
**路径**：`Clock Configuration` 标签页

```
输入时钟源：
HSE: 8 MHz (外部晶振)

PLL配置：
Input frequency: 8 MHz
PLLMUL: x9
PLL output: 72 MHz

系统时钟：
SYSCLK: 72 MHz (最大主频)
HCLK (AHB):  72 MHz
PCLK1 (APB1): 36 MHz (最大36MHz)
PCLK2 (APB2): 72 MHz
```

**重要配置步骤**：
1. 在Clock Configuration页面，HCLK输入框输入：`72`
2. 按回车，CubeMX会自动计算PLL倍频系数
3. 确认各时钟频率无红色警告

---

## 3. SYS系统配置

**路径**：`Pinout & Configuration → System Core → SYS`

| 参数 | 设置值 |
|------|--------|
| **Debug** | Serial Wire (保留SWD调试) |
| **Timebase Source** | SysTick |

⚠️ **重要**：不要选择Disable Debug，否则无法再次烧录程序！

---

## 4. GPIO配置

### 4.1 L298N电机驱动引脚

#### L298N #1 (左侧双电机：左前+左后)

**方向控制引脚**：

| 引脚 | 模式 | 标签 | 说明 |
|------|------|------|------|
| PA0 | GPIO_Output | LF_IN1 | 左前电机方向控制1 |
| PA1 | GPIO_Output | LF_IN2 | 左前电机方向控制2 |
| PA2 | GPIO_Output | LB_IN1 | 左后电机方向控制1 |
| PA3 | GPIO_Output | LB_IN2 | 左后电机方向控制2 |

**配置参数**（PA0/PA1/PA2/PA3）：
- GPIO mode: Output Push Pull
- GPIO Pull-up/Pull-down: No pull-up and no pull-down
- Maximum output speed: Low
- User Label: `LF_IN1` / `LF_IN2` / `LB_IN1` / `LB_IN2`

**L298N#1电机输出连接**：
```
OUT1 → 左前电机 M+ (PA0/PA1控制)
OUT2 → 左前电机 M-
OUT3 → 左后电机 M+ (PA2/PA3控制)
OUT4 → 左后电机 M-
```

#### L298N #2 (右侧双电机：右前+右后)

**方向控制引脚**：

| 引脚 | 模式 | 标签 | 说明 |
|------|------|------|------|
| PA4 | GPIO_Output | RF_IN1 | 右前电机方向控制1 |
| PA5 | GPIO_Output | RF_IN2 | 右前电机方向控制2 |
| PA6 | GPIO_Output | RB_IN1 | 右后电机方向控制1 |
| PA7 | GPIO_Output | RB_IN2 | 右后电机方向控制2 |

**配置参数**（PA4/PA5/PA6/PA7）：
- GPIO mode: Output Push Pull
- GPIO Pull-up/Pull-down: No pull-up and no pull-down
- Maximum output speed: Low
- User Label: `RF_IN1` / `RF_IN2` / `RB_IN1` / `RB_IN2`

**L298N#2电机输出连接**：
```
OUT1 → 右前电机 M+ (PA4/PA5控制)
OUT2 → 右前电机 M-
OUT3 → 右后电机 M+ (PA6/PA7控制)
OUT4 → 右后电机 M-
```

### 4.2 PS2手柄接口

| 引脚 | 模式 | 标签 | 说明 |
|------|------|------|------|
| PB12 | GPIO_Input | PS2_DI | 数据输入 |
| PB13 | GPIO_Output | PS2_CMD | 命令 |
| PB14 | GPIO_Output | PS2_CS | 片选 |
| PB15 | GPIO_Output | PS2_CLK | 时钟 |

**输入引脚配置**（PB12）：
- GPIO mode: Input mode
- GPIO Pull-up/Pull-down: Pull-up
- User Label: `PS2_DI`

**输出引脚配置**（PB13/14/15）：
- GPIO output level: High
- GPIO mode: Output Push Pull
- GPIO Pull-up/Pull-down: No pull-up and no pull-down
- Maximum output speed: High
- User Label: `PS2_CMD` / `PS2_CS` / `PS2_CLK`

### 4.3 用户LED（可选）

| 引脚 | 模式 | 标签 | 说明 |
|------|------|------|------|
| PC13 | GPIO_Output | LED | 板载LED |

---

## 5. TIM1配置（电机PWM输出）

**路径**：`Pinout & Configuration → Timers → TIM1`

⚠️ **重要说明**：TIM1现在配置4个通道，分别控制4个独立电机的PWM

### 5.1 基本配置

| 参数 | 设置值 |
|------|--------|
| **Clock Source** | Internal Clock |
| **Channel1** | PWM Generation CH1 |
| **Channel2** | PWM Generation CH2 |
| **Channel3** | PWM Generation CH3 |
| **Channel4** | PWM Generation CH4 |

### 5.2 Parameter Settings

**Counter Settings**：

| 参数 | 值 | 说明 |
|------|-----|------|
| **Prescaler (PSC)** | 71 | 72MHz/(71+1) = 1MHz |
| **Counter Mode** | Up |
| **Counter Period (ARR)** | 999 | 1MHz/(999+1) = 1kHz PWM频率 |
| **Internal Clock Division** | No Division |
| **Repetition Counter** | 0 |
| **auto-reload preload** | Enable |

**PWM Generation Channel 1**：

| 参数 | 值 |
|------|-----|
| **Mode** | PWM mode 1 |
| **Pulse (CCR1)** | 0 |
| **Fast Mode** | Disable |
| **CH Polarity** | High |
| **Output compare preload** | Enable |

**PWM Generation Channel 2/3/4**：同Channel 1配置

### 5.3 GPIO Settings

确认自动生成的引脚：

| 引脚 | 功能 | 说明 |
|------|------|------|
| PA8 | TIM1_CH1 | 左前电机PWM (LF_ENA) |
| PA9 | TIM1_CH2 | 右前电机PWM (RF_ENA) |
| PA10 | TIM1_CH3 | 左后电机PWM (LB_ENA) |
| PA11 | TIM1_CH4 | 右后电机PWM (RB_ENA) |

**引脚配置**：
- GPIO mode: Alternate Function Push Pull
- Maximum output speed: Low
- User Label: `LF_ENA` / `RF_ENA` / `LB_ENA` / `RB_ENA`

---

## 6. TIM2配置(编码器模式 - 左侧)

**路径**:`Pinout & Configuration → Timers → TIM2`

**说明**:左前和左后电机的编码器信号并联后接入TIM2

### 6.1 基本配置

| 参数 | 设置值 |
|------|--------|
| **Combined Channels** | Encoder Mode |
| **Encoder Mode** | Encoder Mode TI1 and TI2 (四倍频) |

### 6.2 Parameter Settings

**Counter Settings**：

| 参数 | 值 | 说明 |
|------|-----|------|
| **Prescaler** | 0 | 不分频 |
| **Counter Mode** | Up |
| **Counter Period** | 65535 | 16位最大值 |
| **auto-reload preload** | Enable |

**Encoder Settings**：

| 参数 | 值 |
|------|-----|
| **Encoder Mode** | Encoder Mode TI1 and TI2 |
| **Polarity** | |
| - IC1 Polarity | Rising Edge |
| - IC2 Polarity | Rising Edge |
| **Input Filter** | |
| - IC1 Filter | 6 (滤波，减少抖动) |
| - IC2 Filter | 0 |

### 6.3 引脚重映射

⚠️ **重要**：需要使用部分重映射1

**GPIO Settings**：

| 引脚 | 功能 | 说明 |
|------|------|------|
| PA15 | TIM2_CH1 | 左编码器A相 |
| PB3 | TIM2_CH2 | 左编码器B相 |

**配置步骤**：
1. 点击PA15，选择 `TIM2_CH1`
2. 点击PB3，选择 `TIM2_CH2`
3. 在GPIO Settings中确认重映射已启用

**引脚配置**：
- GPIO mode: Input mode
- GPIO Pull-up/Pull-down: Pull-up
- User Label: `ENC_L_A` / `ENC_L_B`

---

## 7. TIM3配置(编码器模式 - 右侧)

**路径**:`Pinout & Configuration → Timers → TIM3`

**说明**:右前和右后电机的编码器信号并联后接入TIM3

### 7.1 基本配置

同TIM2配置

### 7.2 Parameter Settings

同TIM2配置

### 7.3 引脚重映射

⚠️ **重要**：使用部分重映射

**GPIO Settings**：

| 引脚 | 功能 | 说明 |
|------|------|------|
| PB4 | TIM3_CH1 | 右编码器A相 |
| PB5 | TIM3_CH2 | 右编码器B相 |

**引脚配置**：
- GPIO mode: Input mode
- GPIO Pull-up/Pull-down: Pull-up
- User Label: `ENC_R_A` / `ENC_R_B`

---

## 8. TIM4配置（舵机PWM输出）

**路径**：`Pinout & Configuration → Timers → TIM4`

⚠️ **重要说明**：TIM4专门用于控制4个舵机，频率50Hz，脉宽范围0.5ms~2.5ms

### 8.1 基本配置

| 参数 | 设置值 |
|------|--------|
| **Clock Source** | Internal Clock |
| **Channel1** | PWM Generation CH1 |
| **Channel2** | PWM Generation CH2 |
| **Channel3** | PWM Generation CH3 |
| **Channel4** | PWM Generation CH4 |

### 8.2 Parameter Settings

**Counter Settings**：

| 参数 | 值 | 说明 |
|------|-----|------|
| **Prescaler (PSC)** | 71 | 72MHz/(71+1) = 1MHz |
| **Counter Mode** | Up |
| **Counter Period (ARR)** | 19999 | 1MHz/(19999+1) = 50Hz (20ms周期) |
| **Internal Clock Division** | No Division |
| **Repetition Counter** | 0 |
| **auto-reload preload** | Enable |

**PWM Generation Channel 1**：

| 参数 | 值 |
|------|-----|
| **Mode** | PWM mode 1 |
| **Pulse (CCR1)** | 1500 | 初始脉宽1.5ms (90度中位) |
| **Fast Mode** | Disable |
| **CH Polarity** | High |
| **Output compare preload** | Enable |

**PWM Generation Channel 2/3/4**：同Channel 1配置

### 8.3 GPIO Settings

确认自动生成的引脚：

| 引脚 | 功能 | 说明 |
|------|------|------|
| PB6 | TIM4_CH1 | 舵机1 PWM (UP/DOWN键控制) |
| PB7 | TIM4_CH2 | 舵机2 PWM (LEFT/RIGHT键控制) |
| PB8 | TIM4_CH3 | 舵机3 PWM (△/×键控制) |
| PB9 | TIM4_CH4 | 舵机4 PWM (□/○键控制) |

**引脚配置**：
- GPIO mode: Alternate Function Push Pull
- Maximum output speed: Low
- User Label: `SERVO1_PWM` / `SERVO2_PWM` / `SERVO3_PWM` / `SERVO4_PWM`

**舵机PWM计算**：
```
PWM频率 = 1MHz / (19999+1) = 50Hz (周期20ms)
脉宽范围: 500~2500 (对应0.5ms~2.5ms)
角度映射: 0度=500us, 90度=1500us, 180度=2500us
```

---

## 9. USART3配置（串口通信）

**路径**：`Pinout & Configuration → Connectivity → USART3`

### 9.1 基本配置

| 参数 | 设置值 |
|------|--------|
| **Mode** | Asynchronous |
| **Hardware Flow Control** | Disable |

### 9.2 Parameter Settings

**Basic Parameters**：

| 参数 | 值 |
|------|-----|
| **Baud Rate** | 115200 Bits/s |
| **Word Length** | 8 Bits (including Parity) |
| **Parity** | None |
| **Stop Bits** | 1 |
| **Data Direction** | Receive and Transmit |
| **Over Sampling** | 16 Samples |

### 9.3 GPIO Settings

| 引脚 | 功能 | 说明 |
|------|------|------|
| PB10 | USART3_TX | 串口发送 |
| PB11 | USART3_RX | 串口接收 |

**引脚配置**：
- PB10: Alternate Function Push Pull, High speed
- PB11: Input mode, No pull-up and no pull-down
- User Label: `UART_TX` / `UART_RX`

### 9.4 NVIC Settings

**启用中断**：
- ☑ **USART3 global interrupt**
- Preemption Priority: 0
- Sub Priority: 0

---

## 10. 完整引脚分配表

### 10.1 引脚总览

| 引脚 | 功能 | 模式 | 标签 | 说明 |
|------|------|------|------|------|
| **电源和调试** |
| VBAT | Power | - | - | 备用电池 |
| VDD | Power | - | - | 3.3V电源 |
| VSS | Power | - | - | GND |
| PA13 | SWDIO | Debug | - | SWD数据 |
| PA14 | SWCLK | Debug | - | SWD时钟 |
| **电机驱动** |
| PA0 | GPIO_Output | Output PP | LF_IN1 | 左前电机方向1 |
| PA1 | GPIO_Output | Output PP | LF_IN2 | 左前电机方向2 |
| PA2 | GPIO_Output | Output PP | LB_IN1 | 左后电机方向1 |
| PA3 | GPIO_Output | Output PP | LB_IN2 | 左后电机方向2 |
| PA4 | GPIO_Output | Output PP | RF_IN1 | 右前电机方向1 |
| PA5 | GPIO_Output | Output PP | RF_IN2 | 右前电机方向2 |
| PA6 | GPIO_Output | Output PP | RB_IN1 | 右后电机方向1 |
| PA7 | GPIO_Output | Output PP | RB_IN2 | 右后电机方向2 |
| PA8 | TIM1_CH1 | AF PP | LF_ENA | 左前电机PWM |
| PA9 | TIM1_CH2 | AF PP | RF_ENA | 右前电机PWM |
| PA10 | TIM1_CH3 | AF PP | LB_ENA | 左后电机PWM |
| PA11 | TIM1_CH4 | AF PP | RB_ENA | 右后电机PWM |
| **编码器** |
| PA15 | TIM2_CH1 | Input PU | ENC_L_A | 左编码器A |
| PB3 | TIM2_CH2 | Input PU | ENC_L_B | 左编码器B |
| PB4 | TIM3_CH1 | Input PU | ENC_R_A | 右编码器A |
| PB5 | TIM3_CH2 | Input PU | ENC_R_B | 右编码器B |
| **串口** |
| PB10 | USART3_TX | AF PP | UART_TX | 串口发送 |
| PB11 | USART3_RX | Input | UART_RX | 串口接收 |
| **舵机控制** |
| PB6 | TIM4_CH1 | AF PP | SERVO1_PWM | 舵机1 PWM |
| PB7 | TIM4_CH2 | AF PP | SERVO2_PWM | 舵机2 PWM |
| PB8 | TIM4_CH3 | AF PP | SERVO3_PWM | 舵机3 PWM |
| PB9 | TIM4_CH4 | AF PP | SERVO4_PWM | 舵机4 PWM |
| **PS2手柄** |
| PB12 | GPIO_Input | Input PU | PS2_DI | 数据输入 |
| PB13 | GPIO_Output | Output PP | PS2_CMD | 命令 |
| PB14 | GPIO_Output | Output PP | PS2_CS | 片选 |
| PB15 | GPIO_Output | Output PP | PS2_CLK | 时钟 |
| **其他** |
| PC13 | GPIO_Output | Output PP | LED | 用户LED |

### 10.2 引脚图示

```
        STM32F103C8T6 (LQFP48)
      ╔════════════════════════════════╗
VBAT─1┤                                ├48─VDD
PC13─2┤ LED                        GND ├47─GND
   ...                                  ...
LF_IN1─9┤PA0                      PB12├28─PS2_DI
LF_IN2─10┤PA1                      PB13├29─PS2_CMD
LB_IN1─11┤PA2                      PB14├30─PS2_CS
LB_IN2─12┤PA3                      PB15├31─PS2_CLK
RF_IN1─14┤PA4                       PB9├32─SERVO4
RF_IN2─15┤PA5                       PB8├33─SERVO3
RB_IN1─16┤PA6                       PB7├34─SERVO2
RB_IN2─17┤PA7                       PB6├35─SERVO1
   ...                                  ...
LF_ENA─29┤PA8/TIM1_1          PB3/TIM2_2├39─ENC_L_B
RF_ENA─30┤PA9/TIM1_2          PB4/TIM3_1├40─ENC_R_A
LB_ENA─31┤PA10/TIM1_3         PB5/TIM3_2├41─ENC_R_B
RB_ENA─32┤PA11/TIM1_4                   ├...
ENC_L_A─38┤PA15/TIM2_1                   ├...
    ...                                  ...
UART_TX─43┤PB10/USART3_TX               ├...
UART_RX─44┤PB11/USART3_RX               ├...
      ╚════════════════════════════════╝
```

---

## 11. 项目管理器配置

**路径**：`Project Manager` 标签页

### 11.1 Project Settings

| 参数 | 值 |
|------|-----|
| **Project Name** | PS2_Hybird |
| **Project Location** | 你的工作目录 |
| **Toolchain/IDE** | 选择你使用的IDE： |
| | - Makefile (推荐，通用) |
| | - MDK-ARM V5 (Keil) |
| | - STM32CubeIDE |

### 11.2 Code Generator

**代码生成选项**：

- ☑ **Copy only the necessary library files**
- ☑ **Generate peripheral initialization as a pair of '.c/.h' files per peripheral**
- ☑ **Keep User Code when re-generating**
- ☑ **Delete previously generated files when not re-generated**

**HAL Settings**：
- ☑ **Set all free pins as analog (to optimize power consumption)**

---

## 12. 生成代码

### 12.1 生成步骤

1. 检查所有配置无红色警告
2. 点击右上角 **GENERATE CODE** 按钮
3. 或者：`Project → Generate Code`
4. 等待代码生成完成

### 12.2 生成后的目录结构

```
PS2_Hybird/
├── Core/
│   ├── Inc/         # 头文件
│   │   ├── main.h
│   │   ├── gpio.h
│   │   ├── tim.h
│   │   ├── usart.h
│   │   └── ...
│   └── Src/         # 源文件
│       ├── main.c
│       ├── gpio.c
│       ├── tim.c
│       ├── usart.c
│       └── ...
├── Drivers/         # HAL库
│   ├── CMSIS/
│   └── STM32F1xx_HAL_Driver/
└── Makefile         # 编译脚本
```

---

## 13. 验证配置

### 13.1 检查时钟树

在 **Clock Configuration** 页面检查：

```
✓ SYSCLK = 72 MHz
✓ AHB = 72 MHz
✓ APB1 = 36 MHz
✓ APB2 = 72 MHz
✓ TIM1 Clock = 72 MHz (电机PWM)
✓ TIM2/3 Clock = 72 MHz (编码器)
✓ TIM4 Clock = 72 MHz (舵机PWM)
✓ USART3 Clock = 36 MHz
```

### 13.2 检查引脚冲突

在 **Pinout view** 中：
- ❌ 没有红色引脚（冲突）
- ⚠️ 没有黄色引脚（警告）
- ✓ 所有引脚都正确分配

### 13.3 检查中断优先级

在 **NVIC Configuration** 中：
- USART3 global interrupt: Priority 0

---

## 14. 常见问题

### Q1: PA15/PB3不能选择TIM2？
**A**: 需要先禁用JTAG：
1. System Core → SYS
2. Debug 选择: **Serial Wire**
3. 这样会释放PA15, PB3, PB4用于TIM

### Q2: PWM频率如何计算？
**A**: 

**电机PWM (TIM1)**：
```
PWM频率 = 定时器时钟 / (PSC+1) / (ARR+1)
        = 72MHz / (71+1) / (999+1)
        = 1kHz
```

调整ARR可改变频率：
- ARR=999 → 1kHz
- ARR=1999 → 500Hz
- ARR=499 → 2kHz

**舵机PWM (TIM4)**：
```
PWM频率 = 72MHz / (71+1) / (19999+1)
        = 50Hz (周期20ms)
脉宽范围: 500~2500 (0.5ms~2.5ms)
```

### Q3: 编码器计数不准确？
**A**: 
1. 确认使用了 Encoder Mode TI1 and TI2（四倍频）
2. 添加输入滤波（IC1 Filter = 6）
3. 使用上拉电阻

### Q4: 串口无法通信？
**A**: 
1. 确认波特率115200
2. 检查TX/RX是否接反
3. 确认启用了USART3中断
4. 检查时钟配置（APB1 = 36MHz）

### Q5: 舵机抖动或不工作？
**A**: 
1. 确认TIM4配置：PSC=71, ARR=19999 (50Hz)
2. 检查脉宽范围：500~2500us
3. 确认舵机供电充足（5V/3A独立供电）
4. 检查PWM引脚是否正确（PB6-PB9）
5. 启动PWM：`HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_x)`

### Q6: 四个电机无法独立控制？
**A**: 
1. 确认TIM1配置了4个通道（CH1-CH4）
2. 检查引脚映射：PA8/PA9/PA10/PA11
3. 确认每个电机的方向GPIO配置正确（PA0-PA7）
4. 检查L298N接线是否对应代码中的引脚定义

---

## 15. 导出和导入

### 15.1 保存.ioc文件
- CubeMX配置保存在 `PS2_Hybird.ioc` 文件中
- 定期备份此文件

### 15.2 重新打开项目
1. File → Open Project
2. 选择 `PS2_Hybird.ioc`
3. 修改配置后重新生成代码

### 15.3 分享配置
- 将 `.ioc` 文件发送给团队成员
- 确保使用相同版本的CubeMX

---

## 16. 硬件连接检查清单

### 16.1 电机系统

**左前电机 (LF)**：
- ✅ 方向控制：PA0 → L298N#1 IN1, PA1 → L298N#1 IN2
- ✅ PWM控制：PA8 → L298N#1 ENA
- ✅ 电机输出：L298N#1 OUT1/OUT2

**左后电机 (LB)**：
- ✅ 方向控制：PA2 → L298N#1 IN3, PA3 → L298N#1 IN4
- ✅ PWM控制：PA10 → L298N#1 ENB
- ✅ 电机输出：L298N#1 OUT3/OUT4

**右前电机 (RF)**：
- ✅ 方向控制：PA4 → L298N#2 IN1, PA5 → L298N#2 IN2
- ✅ PWM控制：PA9 → L298N#2 ENA
- ✅ 电机输出：L298N#2 OUT1/OUT2

**右后电机 (RB)**：
- ✅ 方向控制：PA6 → L298N#2 IN3, PA7 → L298N#2 IN4
- ✅ PWM控制：PA11 → L298N#2 ENB
- ✅ 电机输出：L298N#2 OUT3/OUT4

### 16.2 舵机系统

- ✅ 舵机1信号线：PB6 (TIM4_CH1)
- ✅ 舵机2信号线：PB7 (TIM4_CH2)
- ✅ 舵机3信号线：PB8 (TIM4_CH3)
- ✅ 舵机4信号线：PB9 (TIM4_CH4)
- ✅ 舵机供电：5V/3A独立电源（AMS1117-5.0）
- ✅ 舵机GND与STM32 GND共地

### 16.3 编码器系统

- ✅ 左编码器：PA15 (A相), PB3 (B相)
- ✅ 右编码器：PB4 (A相), PB5 (B相)
- ✅ 编码器供电：3.3V
- ✅ 上拉配置：软件上拉或外部上拉

### 16.4 PS2手柄接口

- ✅ DI数据输入：PB12 (上拉输入)
- ✅ CMD命令：PB13 (推挽输出)
- ✅ CS片选：PB14 (推挽输出)
- ✅ CLK时钟：PB15 (推挽输出)
- ✅ PS2供电：3.3V/500mA (RT92013-33GB)

### 16.5 电源系统

- ✅ STM32+PS2：3.3V/500mA (RT92013-33GB)
- ✅ 舵机：5V/3A (AMS1117-5.0)
- ✅ 电机驱动：6V/3A (LM2596模块)
- ✅ 所有GND连接在一起

---

## 17. 下一步

配置完成后：

1. ✅ 生成代码
2. ✅ 复制 `ax_ps2.c/h` 到项目
3. ✅ 复制 `motor_control.c/h` 到项目
4. ✅ 复制 `servo_control.c/h` 到项目
5. ✅ 修改 `main.c` 添加控制逻辑
6. ✅ 修改 `usart.c` 添加串口协议
7. ✅ 编译项目（`make all`）
8. ✅ 下载到MCU
9. ✅ 测试电机功能（前后左右平移+旋转）
10. ✅ 测试舵机功能（PS2按键控制角度）
11. ✅ 调试混合控制（同时控制电机和舵机）

---

## 📚 参考资料

- [STM32CubeMX用户手册](https://www.st.com/resource/en/user_manual/dm00104712.pdf)
- [STM32F103参考手册](https://www.st.com/resource/en/reference_manual/cd00171190.pdf)
- [HAL库使用手册](https://www.st.com/resource/en/user_manual/dm00105879.pdf)

---

**配置版本**：STM32CubeMX 6.x  
**芯片**：STM32F103C8T6  
**项目**：PS2麦克纳姆轮+四舵机混合控制系统  
**硬件配置**：
- 4个麦克纳姆轮独立电机 + 2个L298N电驱
- 4个MG996R舵机独立PWM控制
- PS2无线手柄控制
- 双编码器速度反馈
- 三路独立供电系统
**更新日期**：2025-11-01
