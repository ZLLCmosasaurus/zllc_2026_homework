# DJI-3508电机CAN通信与PID速度控制使用说明

## 概述

本项目实现了通过CAN总线控制DJI-3508电机，并使用PID算法进行速度闭环控制的功能。该实现基于STM32F1系列微控制器和HAL库。

## 功能特性

1. **CAN通信控制**：通过CAN总线发送控制指令给DJI-3508电机
2. **PID速度控制**：实现速度闭环控制，精确控制电机转速
3. **多电机支持**：可同时控制多达4个DJI-3508电机
4. **反馈数据处理**：接收并解析电机反馈数据

## 硬件连接

- **CAN总线连接**：
  - PA11 (CAN_RX)
  - PA12 (CAN_TX)
  
- **终端电阻**：确保CAN总线上有120欧姆终端电阻

## 软件架构

### 主要文件

1. **can.h/can.c**：CAN通信相关函数实现
2. **main.c**：主程序，包含PID控制逻辑
3. **stm32f1xx_it.c**：中断处理程序

### 核心函数

#### 1. 电机控制函数

```c
// 控制单个电机
void can_dji_motor_control(uint8_t motor_id, int16_t current);

// 同时控制四个电机
void can_dji_motor_control_multi(int16_t current1, int16_t current2, int16_t current3, int16_t current4);
```

#### 2. PID控制函数

```c
// 初始化PID控制器
void pid_init(pid_controller_t *pid, float kp, float ki, float kd);

// PID计算
float pid_calculate(pid_controller_t *pid, float target, float current);
```

## 使用方法

### 1. 初始化

```c
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 4;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_2TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_3TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_5TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;

  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }

      CAN_FilterTypeDef can_filter;
    can_filter.FilterBank           = 0;
    can_filter.FilterMode           = CAN_FILTERMODE_IDMASK;
    can_filter.FilterScale          = CAN_FILTERSCALE_32BIT;
    can_filter.FilterIdHigh         = 0x0000;
    can_filter.FilterIdLow          = 0x0000;
    can_filter.FilterMaskIdHigh     = 0x0000;
    can_filter.FilterMaskIdLow      = 0x0000;
    can_filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    can_filter.FilterActivation     = ENABLE;
    can_filter.SlaveStartFilterBank = 14;

    if (HAL_CAN_ConfigFilter(&hcan, &can_filter) != HAL_OK) 
    {
        Error_Handler();
    }

    if (HAL_CAN_Start(&hcan) != HAL_OK) 
    {
        Error_Handler();
    }

    if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
        Error_Handler();
    }
```

### 2. 参数配置

```c
// PID参数设置
pid_init(&motor_pid, 10.0f, 0.1f, 1.0f);  

// 目标速度设置
motor_pid.target = 1000.0f;  // 目标速度为1000 RPM
```

### 3. 控制流程

1. 系统每10ms执行一次PID计算
2. 获取电机实际速度（在实际应用中应从电机反馈数据中获取）
3. 使用PID算法计算输出电流
4. 通过CAN总线发送控制指令给电机

## CAN通信协议

### 控制指令帧

- **CAN ID**：0x200
- **数据长度**：8字节
- **数据格式**：
  - 字节0-1：电机1电流值（高字节在前）
  - 字节2-3：电机2电流值（高字节在前）
  - 字节4-5：电机3电流值（高字节在前）
  - 字节6-7：电机4电流值（高字节在前）

### 反馈数据帧

- **CAN ID**：0x201 ~ 0x204（对应电机1~4）
- **数据长度**：8字节
- **数据格式**：
  - 字节0-1：电机角度（高字节在前）
  - 字节2-3：电机转速（高字节在前）
  - 字节4-5：电机电流（高字节在前）
  - 字节6-7：电机温度（高字节在前）

### 1. PID控制算法理解

#### 1.1 PID基本概念
PID控制器由三个部分组成：
- **比例项(P)**：根据当前误差大小进行调整
- **积分项(I)**：根据误差累积进行调整，消除稳态误差
- **微分项(D)**：根据误差变化率进行调整，预测未来趋势

#### 1.2 PID数学表达式
```
输出 = Kp * 误差 + Ki * ∫误差dt + Kd * d(误差)/dt
```

#### 1.3 PID参数调节经验
- 先调Kp，再调Kd，最后调Ki
- Kp过大容易引起振荡，过小响应慢
- Ki过大容易引起积分饱和和振荡
- Kd有助于减少超调和提高稳定性

## PID参数调节建议

1. **比例系数(Kp)**：决定系统响应速度，值越大响应越快但可能产生振荡
2. **积分系数(Ki)**：消除稳态误差，值过大会导致系统不稳定
3. **微分系数(Kd)**：抑制超调，提高系统稳定性

建议调节顺序：Kp → Kd → Ki

## 注意事项

1. 确保CAN总线物理连接正确且有终端电阻
2. 电机电流限制在±16384范围内
3. 实际应用中应从电机反馈数据中获取真实的速度值
4. 根据具体应用场景调整PID参数

### 3. 代码分析与优化

#### 3.1 发现的问题
1. **积分饱和问题**：积分项没有适当限制，导致电机持续加速
2. **微分项计算不准确**：未考虑采样时间间隔
3. **参数设置不合理**：初始PID参数不适合实际控制场景
4. **角度控制逻辑缺陷**：使用单圈角度而非连续角度

#### 3.2 解决方案
1. **修复积分饱和**：
   - 添加积分限幅机制
   - 积分项乘以采样时间间隔

2. **改进微分项计算**：
   - 微分项除以采样时间间隔
   - 在主循环中计算实际时间间隔

3. **优化PID参数**：
   - 调整为更适合实际控制的参数
   - 角度环：Kp=2.0, Ki=0.01, Kd=0.1
   - 速度环：Kp=2.0, Ki=0.01, Kd=0.1

4. **改善角度控制**：
   - 使用连续角度进行计算
   - 实现更平滑的比例控制策略
