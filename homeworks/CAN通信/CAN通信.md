# 2.1.1 学习日志

## CAN 通信配置

### 基本参数设置

CAN 通信大部分的配置 CubeMX 已经准备好了，配置带宽为 1Mbps，然后只需要配置过滤器以及收到报文后的处理函数即可。

### 报文接收

有 RoboMaster 官方文档可知。C620 电调反馈的报文为标准帧，格式为数据帧，标识符为`0x200` + 电调 ID，DLC 为 8 个字节。

#### 过滤器配置

由于报文是多个 ID，此处只用到的一种类型，因此选用 32 位掩码模式的过滤器。

设置 ID 高 16 位为`0x200 << 5`，低 16 位为`0x0000`，掩码高 16 位为`0x7F0 << 5`，低 16 位为`0x0000 | 0x02 | 0x04`。

这样就配置好了只接受标准帧格式的`0x20`开头的数据帧。

#### 数据解析

<img src="./images/df3d70a3-2882-40ef-ba84-96dfe4485a1f.png" title="" alt="df3d70a3-2882-40ef-ba84-96dfe4485a1f" style="zoom:50%;">

代码如下所示

```c
typedef struct
{
    uint8_t id;
    uint16_t angle;
    uint16_t speed;
    float current;
    uint8_t temperature;
} C620_Motor_Status_TypeDef;

float convert_raw_to_angle(uint16_t angle)
{
    const float SCALE_FACTOR = 360.0f / 8191.0f;
    return raw * SCALE_FACTOR;
}

float convert_raw_to_amps(int16_t raw)
{
    const float SCALE_FACTOR = 20.0f / 16384.0f;
    return raw * SCALE_FACTOR;
}

void C620_Motor_Status_Init(C620_Motor_Status_TypeDef *status, uint32_t stdId, uint8_t *data)
{
    status->id          = stdId & 0x00F;
    status->angle       = convert_raw_to_angle((data[0] << 8) | data[1]);
    status->speed       = (data[2] << 8) | data[3];
    status->current     = convert_raw_to_amps((data[4] << 8) | data[5]);
    status->temperature = data[6];
}
```

### 报文发送

根据官方文档，报文的接受使用了两个标识符，分别是`0x200`与`0x1FF`，分别控制 1-4 号与 5-8 号电调。

帧格式为标准帧的数据帧，DLC 为 8 个字节，每个电调使用两个字节，分别为控制电流的高八位和低八位。

#### 数据解析

控制电流为两个字节，范围是`-16384~0~16384`，对应电调输出的转矩电流范围为`-20~0~20A`。

```c
typedef struct
{
    uint8_t id;
    float current;
} C620_Motor_Control_Typedef;

int16_t convert_amps_to_raw(float current_amps)
{
    const float SCALE_FACTOR = 16384.0f / 20.0f;
    return (int16_t)roundf(current_amps * SCALE_FACTOR);
}

void C620_Motor_Control_Init(C620_Motor_Control_Typedef *control, uint8_t *data)
{
    uint8_t start   = (control->id > 4) ? ((control->id - 4) * 2 - 1) : (control->id * 2 - 2);
    uint16_t raw    = convert_amps_to_raw(control->current);
    data[start]     = (raw & 0xFF00) >> 8;
    data[start + 1] = raw & 0x00FF;
}
```

## PID 配置

### PID 运算相关

此处选用的是位置式 PID。除了位置 PID 外，还有增量式 PID。
下面的代码还包括了输出的限位，以确保设备的安全。

```c
typedef struct
{
    float Kp, Ki, Kd;
    float target, actual;
    float err, last_err, total_err;
    float output, output_max, output_min;
} PID_TypeDef;

void PID_Init(PID_TypeDef *pid, float Kp, float Ki, float Kd, float output_max, float output_min)
{
    pid->Kp         = Kp;
    pid->Ki         = Ki;
    pid->Kd         = Kd;
    pid->target     = 0;
    pid->actual     = 0;
    pid->err        = 0;
    pid->last_err   = 0;
    pid->total_err  = 0;
    pid->output     = 0;
    pid->output_max = output_max;
    pid->output_min = output_min;
}

float PID_Calculate(PID_TypeDef *pid, float actual)
{
    pid->actual = actual;
    pid->err    = pid->target - pid->actual;
    pid->total_err += pid->err;
    pid->output   = pid->Kp * pid->err + pid->Ki * pid->total_err + pid->Kd * (pid->err - pid->last_err);
    pid->last_err = pid->err;
    if (pid->output < pid->output_min) {
        pid->output = pid->output_min;
    }
    if (pid->output > pid->output_max) {
        pid->output = pid->output_max;
    }
    return pid->output;
}
```

## 具体实现思路

用一个`Controller`来统管所有与电机有关的数据和函数。大致结构如下。

```c
void Controller_Init();

void CAN_RxHandler(uint32_t stdId, const uint8_t *rx_buff);
void EXTI_Handler(int16_t GPIO_Pin);

void C620_Motor_Speed_PID_Update(int16_t *speed);
void C620_Motor_Angle_PID_Update(int16_t *angles);
```

### 数据的接受

在 CAN 通信的接受中断函数中，以及外部中断的函数中，分别调用`CAN_RxHandler`和`EXTI_Handler`函数，并将参数传递过去，以此来尽可能减少代码的过度耦合，尽可能让实现各自功能的代码仅关注自身功能，避免业务逻辑的分散、增加代码的管理难度。

```c
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rxHeader;
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, CAN_RxBuff);

    CAN_RxHandler(rxHeader.StdId, CAN_RxBuff);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    EXTI_Handler(GPIO_Pin);
}
```

### 多电机的支持

用一个 8 位的常量来保存 8 个电机的状态，0 为关，1 为开。此后所有控制的地方在对这个量做位运算来判断是否需要处理。

```c
const uint8_t MOTOR_ENABLE = 0x01;

for (size_t i = 0; i < 8; i++) {
    if (MOTOR_ENABLE & (1 << i)) {
        // TODO 要对单个电机执行的操作
    }
}
```

### 角度环对于多圈位置控制的支持

CAN 报文反馈的数据中，角度只有 0 ～ 360 度，但我们要实现-720 ～ 720 度的位置控制。此时则需要记录上一刻的角度数据，利用 0 与 360 的临界点，用一个新的变量存储转动圈数。当差值大于一定范围时，说明是从 360 度变为 0 度，为倒转，圈数减少；反之，圈数增加。具体代码实现如下。

```c
static float last_angle[8]  = {};
static int16_t spin_n[8]    = {};

float angle = status->angle;
if (angle - last_angle[id] > 180.0f) {
    spin_n[id]--;
}
if (angle - last_angle[id] < -180.0f) {
    spin_n[id]++;
}

last_angle[id] = angle;
```

此后再向 PID 发送`actual`数据时，实际发送的为`spin_n[id] * 360.0f + angle`。

## 调试工具

### JLink + Ozone

由于本人无 JLink，便不再阐述此方法。

### Vofa+

使用`JustFloat`模式，将电机反馈的数据发送到上位机。

创建`debug.c/h`文件封装上述操作。

```c
#include "debug.h"
#include "usart.h"

const uint8_t tail[4] = {0x00, 0x00, 0x80, 0x7f};
uint8_t tx_step       = 0; // 0: 空闲, 1: 正在发送数据, 2: 正在发送 tail

void print_debug(float *data, size_t length)
{
    tx_step = 1;
    HAL_UART_Transmit_DMA(&huart1, (uint8_t *)data, length * sizeof(float));
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == huart1.Instance) {
        if (tx_step == 1) {
            // 步骤 1 完成：浮点数据发送完毕
            tx_step = 2; // 标记为发送 tail

            // 启动第二次 DMA 传输：发送 tail
            HAL_UART_Transmit_DMA(&huart1, tail, 4);
        } else if (tx_step == 2) {
            // 步骤 2 完成：tail 数据发送完毕
            tx_step = 0; // 标记为完全空闲，可以开始下一个任务
        }
    }
}

```

此处使用了`HAL_UART_TxCpltCallback`函数，原因在于串口的发送是异步的，过快的发送数据会导致帧尾来不及发出。

### 模拟算法

在用实体硬件调试前，我先编写了一部分仿真代码测试 PID。一个好的仿真算法可以节省大量的调试时间。具体算法如下。

```c
// 模拟电机转动速度反馈
static uint32_t time     = 0;
uint32_t delta_time      = HAL_GetTick() - time;
time                     = HAL_GetTick();
int16_t raw              = (rx_buff[0] << 8) | rx_buff[1];
const float SCALE_FACTOR = 20.0f / 16384.0f;
float last_speed         = motor_status[0].speed;
motor_status[0].speed    = (int16_t)(raw * SCALE_FACTOR * 10);
motor_status[0].angle += (last_speed + motor_status[0].speed) * delta_time * 3 / 1000;
motor_status[0].angle = fmodf(motor_status[0].angle, 360.0f);
motor_status[0].angle = (motor_status[0].angle < 0) ? (motor_status[0].angle + 360.0f) : motor_status[0].angle;
```

应用时，添加对`0x200`ID 的报文接受过滤器，然后在接受中断函数中调用上述代码即可。

## 实验现象

[速度环 Vofa+调试视频](./videos/速度环Vofa.mp4)

[速度环实机调试视频](./videos/速度环.mp4)

[角度环 Vofa+调试视频](./videos/角度环Vofa.mp4)

[角度环调试视频](./videos/角度环.mp4)

## 总结与反思

1. 学习了 CAN 通信的相关知识与具体应用。CAN 通信的各个功能和实现都十分惊艳，需要仔细学习与理解。
1. 对于项目开发，代码的管理极为重要，尽可能让各个函数、各个文件各司其职。
1. 调试工具十分重要，尤其是对于需要依靠经验和实践确定参数值的常数确定，一个可视化的、即时的工具大有帮助。
1. PID 调参经验不足。调参过程中，角度环在目标位置附近剧烈震荡问题难以解决。
1. 一个好的仿真算法可以辅助 PID 调试
1. 没有使用 Ozone 调试，之后会自行学习。
