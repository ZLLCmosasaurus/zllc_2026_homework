# 仓库代码控制小车 作业笔记

## 仓库代码解析

战队仓库中的代码高度解耦，各部分各司其职，使得代码可读性很高。从`main.c`中出发，可以发现在所有外设初始化后，便执行`Task_Init()`函数中。这个函数位于`tsk_config_and_callback.h`文件中，也是我们要改写的主要部分。然后再主循环函数中，会持续不断的调用`Task_Loop()`函数。

### `Task_Init()`函数解析

如果说`main()`函数是对外设的初始化，那么在`Task_Init()`函数中，则是对相比于外设更向上、更抽象一层的初始化。例如底盘类便是对包括电机等一系列外设的统筹者，而其初始化便是在该函数中进行。 `Task_Init()`函数还包括对事件回调函数的初始化，例如 CAN 接受事件、UART 接受事件、计时器中断事件等。在回调函数中，便要根据接收到的数据，进行数据的解析和处理，并可能调用其他更多事件处理函数。例如在 DR16 遥控器的 UART 接受中断事件中，除了对数据本身的处理，还包括了进一步的控制策略处理。当然，尽管如此，代码解耦度依旧相当高，`dvc_dr16.h`中，只有对数据的处理，而不包含其他事件的调用，这部分的调用在 UART 接受中断事件中。可以说，几乎所有的代码只需要关注自身要做的事，而无需关注对更进一步的细节处理。

### `dvc_dr16.h/cpp`文件解析

在头文件中，主要包括了一些按键的宏定义和枚举定义，以及 DR16 类的定义。其中大部分只是单纯的各种 Get 函数，但也有值得注意和学习的部分。

#### 保存上一次状态及突变的处理

注意到在`DR16_UART_RxCpltCallback`函数中，在调用`DR16_Data_Process`函数后，还有一句代码用于保存这上一次的数据。

```c
memcpy(&Pre_UART_Rx_Data, UART_Manage_Object_1->Rx_Buffer, sizeof(Struct_DR16_UART_Data));
```

然后在`DR16_Data_Process`中，会调用`Judge_Switch`以及`Judge_Key`函数，在其中与上一次状态比较，得到这次的按键状态是突变还是稳定状态。相对应的，状态的枚举类型中也有突变状态的定义。

#### 在线状态的判断

在`TIM1msMod50_Alive_PeriodElapsedCallback`函数中，会通过判断`Data_Flag`是否发生变化来判断 DR16 的连接状态。

### `dvc_djimotor.h/cpp`文件解析

#### CAN 发送字节指针的分配

在`allocate_tx_data`函数中，会根据 ID 将 CAN 发送字节数组的对应位置的指针赋给成员变量，方便控制。

#### 多种控制模式的指定

在初始化函数中，可以指定想要的控制模式，例如开环、扭矩、角速度等。也可以调用 Set 函数再改变。

#### PID 控制

有两种 PID 类成员，分别是`PID_Omega`和`PID_Angle`。两者的区别在于单位不同。可以调用`Set_Target_Omega_Radian`和`Set_Target_Omega_Angle`函数来设定目标值，然后通过调用`TIM_PID_PeriodElapsedCallback`函数，进行根据控制类型统一进行 PID 的运算。在使用前需要对 PID 成员进行初始化，通过调用相应的`Init`函数来设定一些常量的值。

## 实际代码编写

### 一些全局变量

- DR16 类成员
- 4 个电机类成员
- 电机控制状态

```cpp
Class_DJI_Motor_C620 Motor_Wheel[4];
Class_DR16 DR16;
Enum_Chassis_Control_Type Chassis_Control_Type = Chassis_Control_Type_DISABLE;
```

### DR16 串口的中断回调函数

直接调用 DR16 自身的中断回调函数处理即可，只是单纯的数据解析。

```c
void DR16_UART3_Callback(uint8_t *Buffer, uint16_t Length)
{
    DR16.DR16_UART_RxCpltCallback(Buffer);
}
```

### CAN 接受中断函数

也是单纯的数据解析。

```c
void Chassis_Device_CAN1_Callback(Struct_CAN_Rx_Buffer *CAN_RxMessage)
{
    switch (CAN_RxMessage->Header.StdId)
    {
    case (0x201):
    {
        Motor_Wheel[0].CAN_RxCpltCallback(CAN_RxMessage->Data);
    }
    break;
    case (0x202):
    {
        Motor_Wheel[1].CAN_RxCpltCallback(CAN_RxMessage->Data);
    }
    break;
    case (0x203):
    {
        Motor_Wheel[2].CAN_RxCpltCallback(CAN_RxMessage->Data);
    }
    break;
    case (0x204):
    {
        Motor_Wheel[3].CAN_RxCpltCallback(CAN_RxMessage->Data);
    }
    break;
    }
}
```

### DR16 在线状态判断定时器回调函数

此函数用于判断 DR16 在线状态，并决定底盘的控制状态。

```c
void Reload_TIM_Status_PeriodElapsedCallback()
{
    static uint8_t DR16_STATUS = 0;
    static uint32_t time = 0;

    switch (DR16_STATUS)
    {
    // 离线检测状态
    case (0):
    {
        // 遥控器中途断联导致错误离线 跳转到 遥控器串口错误状态
        if (huart3.ErrorCode)
        {
            time = 0;
            DR16_STATUS = 4;
        }

        // 转移为 在线状态
        if (DR16.Get_DR16_Status() == DR16_Status_ENABLE)
        {
            time = 0;
            DR16_STATUS = 2;
        }

        // 超过一秒的遥控器离线 跳转到 遥控器关闭状态
        if (time > 200)
        {
            time = 0;
            DR16_STATUS = 1;
        }
    }
    break;
    // 遥控器关闭状态
    case (1):
    {
        // 离线保护
        Chassis_Control_Type = Chassis_Control_Type_DISABLE;

        if (DR16.Get_DR16_Status() == DR16_Status_ENABLE)
        {
            time = 0;
            DR16_STATUS = 2;
        }

        // 遥控器中途断联导致错误离线 跳转到 遥控器串口错误状态
        if (huart3.ErrorCode)
        {
            time = 0;
            DR16_STATUS = 4;
        }
    }
    break;
    // 遥控器在线状态
    case (2):
    {
        // 转移为 刚离线状态
        if (DR16.Get_DR16_Status() == DR16_Status_DISABLE)
        {
            time = 0;
            DR16_STATUS = 3;
        }
        else
        {
            Chassis_Control_Type = Chassis_Control_Type_FLLOW;
        }
    }
    break;
    // 刚离线状态
    case (3):
    {
        // 无条件转移到 离线检测状态
        time = 0;
        DR16_STATUS = 0;
    }
    break;
    // 遥控器串口错误状态
    case (4):
    {
        HAL_UART_DMAStop(&huart3); // 停止以重启
        // HAL_Delay(10); // 等待错误结束
        HAL_UARTEx_ReceiveToIdle_DMA(&huart3, UART3_Manage_Object.Rx_Buffer, UART3_Manage_Object.Rx_Buffer_Length);

        // 处理完直接跳转到 离线检测状态
        time = 0;
        DR16_STATUS = 0;
    }
    break;
    }

    time++;
}
```

### 速度解算函数

这一步先判断点击运行状态，根据状态决定是从 DR16 读取控制数据并解析还是让 4 轮全部锁停。

```c
void speed_resolution()
{
    const float Dead_Zone = 0.05;
    const float Velocity_Max = 2.0f;

    if (Chassis_Control_Type == Chassis_Control_Type_FLLOW)
    {
        float dr16_l_x, dr16_l_y, dr16_r_x;
        // 排除遥控器死区
        dr16_l_x = (Math_Abs(DR16.Get_Left_X()) > Dead_Zone) ? DR16.Get_Left_X() : 0;
        dr16_l_y = (Math_Abs(DR16.Get_Left_Y()) > Dead_Zone) ? DR16.Get_Left_Y() : 0;
        dr16_r_x = (Math_Abs(DR16.Get_Right_X()) > Dead_Zone) ? DR16.Get_Right_X() : 0;

        // 设定矩形到圆形映射进行控制
        float chassis_velocity_x = dr16_l_x * sqrt(1.0f - dr16_l_y * dr16_l_y / 2.0f) * Velocity_Max;
        float chassis_velocity_y = dr16_l_y * sqrt(1.0f - dr16_l_x * dr16_l_x / 2.0f) * Velocity_Max;

        // 设定角速度
        float chassis_omega = -dr16_r_x * Velocity_Max;

        // 底盘四电机模式配置
        for (int i = 0; i < 4; i++)
        {
            Motor_Wheel[i].Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
        }

        // 速度换算，正运动学分解
        float motor1_temp_linear_vel = chassis_velocity_y - chassis_velocity_x + chassis_omega * (HALF_WIDTH + HALF_LENGTH);
        float motor2_temp_linear_vel = chassis_velocity_y + chassis_velocity_x - chassis_omega * (HALF_WIDTH + HALF_LENGTH);
        float motor3_temp_linear_vel = chassis_velocity_y + chassis_velocity_x + chassis_omega * (HALF_WIDTH + HALF_LENGTH);
        float motor4_temp_linear_vel = chassis_velocity_y - chassis_velocity_x - chassis_omega * (HALF_WIDTH + HALF_LENGTH);

        // 线速度 cm/s  转角速度  RAD
        float motor1_temp_rad = motor1_temp_linear_vel * VEL2RAD;
        float motor2_temp_rad = motor2_temp_linear_vel * VEL2RAD;
        float motor3_temp_rad = motor3_temp_linear_vel * VEL2RAD;
        float motor4_temp_rad = motor4_temp_linear_vel * VEL2RAD;
        // 角速度*减速比  设定目标 直接给到电机输出轴
        Motor_Wheel[0].Set_Target_Omega_Radian(motor2_temp_rad);
        Motor_Wheel[1].Set_Target_Omega_Radian(-motor1_temp_rad);
        Motor_Wheel[2].Set_Target_Omega_Radian(-motor3_temp_rad);
        Motor_Wheel[3].Set_Target_Omega_Radian(motor4_temp_rad);
    }
    else if (Chassis_Control_Type == Chassis_Control_Type_DISABLE)
    {
        // 底盘失能 四轮子自锁
        for (int i = 0; i < 4; i++)
        {
            Motor_Wheel[i].Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
            Motor_Wheel[i].PID_Angle.Set_Integral_Error(0.0f);
            Motor_Wheel[i].Set_Target_Omega_Radian(0.0f);
        }
    }

    // 各个电机具体PID
    for (int i = 0; i < 4; i++)
    {
        Motor_Wheel[i].TIM_PID_PeriodElapsedCallback();
    }
}
```

### 定时器中断回调函数

DR16 自身的状态判断需要一个外界的时钟源以一个不算太高的频率触发。同时，在定时器中，还需要进行运动状态的判断和运动的解算。

```c
void Task1ms_TIM5_Callback()
{
    static uint8_t mod = 0;
    if (++mod == 50)
    {
        DR16.TIM1msMod50_Alive_PeriodElapsedCallback();
        mod = 0;
    }
    Reload_TIM_Status_PeriodElapsedCallback();
    speed_resolution();

    // 统一打包发送
    TIM_CAN_PeriodElapsedCallback();
}
```

### `Task_Init()`函数

此函数用途如上面所讲。

```cpp
extern "C" void Task_Init()
{

    DWT_Init(168);
    // 手柄初始化
    DR16.Init(&huart3, nullptr);

    // 电机PID批量初始化
    for (int i = 0; i < 4; i++)
    {
        Motor_Wheel[i].PID_Omega.Init(1500.0f, 0.0f, 0.0f, 0.0f, Motor_Wheel[i].Get_Output_Max(), Motor_Wheel[i].Get_Output_Max());
    }

    // 轮向电机ID初始化
    Motor_Wheel[0].Init(&hcan1, DJI_Motor_ID_0x201);
    Motor_Wheel[1].Init(&hcan1, DJI_Motor_ID_0x202);
    Motor_Wheel[2].Init(&hcan1, DJI_Motor_ID_0x203);
    Motor_Wheel[3].Init(&hcan1, DJI_Motor_ID_0x204);

    CAN_Init(&hcan1, Chassis_Device_CAN1_Callback);
    UART_Init(&huart3, DR16_UART3_Callback, 18);

    // 定时器循环任务
    TIM_Init(&htim4, Task100us_TIM4_Callback);
    TIM_Init(&htim5, Task1ms_TIM5_Callback);

    HAL_TIM_Base_Start_IT(&htim4);
    HAL_TIM_Base_Start_IT(&htim5);
}
```

## 总结

本次任务主要学习了仓库代码的使用，一方面通过学习他人的代码，可以为自身积攒更多开发技能和经验。另一方面也为将来的工作做了准备。
