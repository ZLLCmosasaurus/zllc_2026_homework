# dji3508电机
## 1.dvc_djimotor.h
开头包含：
大疆状态（控制动与不动）；大疆电机ID的枚举；大疆电机的控制方式；大疆电机源数据（can反馈的原始值）；处理过后的数据
还有一些单个数据，比如：
```c
    //电机上电第一帧标志位
    uint8_t Start_Falg = 0;

    //之后还有这个
    if(Start_Falg==0)  Start_Falg = 1;
```
Q：为什么要有第一帧标志位的设计🧐

A：已知“电机首次启动时，Data.Pre_Encoder是未初始化的垃圾”，这一步可以避免首次数据无效。



##### 说明：class Class_DIJ_Motor_C620  这是关于dji3508电机的类（因为其电调是C620）

## 2.dvc_djimotor.cpp里面（补充；最开始包含allocate_tx_data的定义）
### 1.电机初始化(难点)
注意是一个电机一个电机分别初始化的，这样并不会“打架”。
```c
void Class_DJI_Motor_C620::Init(CAN_HandleTypeDef *hcan, Enum_DJI_Motor_ID __CAN_ID, Enum_DJI_Motor_Control_Method __DJI_Motor_Control_Method, float __Gearbox_Rate, float __Torque_Max)
{
    if (hcan->Instance == CAN1)
    {
        CAN_Manage_Object = &CAN1_Manage_Object;
    }
    else if (hcan->Instance == CAN2)
    {
        CAN_Manage_Object = &CAN2_Manage_Object;
    }
    CAN_ID = __CAN_ID;
    DJI_Motor_Control_Method = __DJI_Motor_Control_Method;
    Gearbox_Rate = __Gearbox_Rate;
    Torque_Max = __Torque_Max;
    this->CAN_Tx_Data = allocate_tx_data(hcan, __CAN_ID);
}
```
核心作用：分配发送缓存区的地址给CAN_Tx_Data

具体逻辑如下：
```c
uint8_t *allocate_tx_data(CAN_HandleTypeDef *hcan, Enum_DJI_Motor_ID __CAN_ID)
{
    uint8_t *tmp_tx_data_ptr;
    if (hcan == &hcan1)
    {
        switch (__CAN_ID)
        {
        case (DJI_Motor_ID_0x201):
        {
            tmp_tx_data_ptr = &(CAN1_0x200_Tx_Data[0]);
        }
        break;
        case (DJI_Motor_ID_0x202):
        {
            tmp_tx_data_ptr = &(CAN1_0x200_Tx_Data[2]);
        }
        break;
        case (DJI_Motor_ID_0x203):
        {
            tmp_tx_data_ptr = &(CAN1_0x200_Tx_Data[4]);
        }
        break;
        case (DJI_Motor_ID_0x204):
        {
            tmp_tx_data_ptr = &(CAN1_0x200_Tx_Data[6]);
        }
        break;
        }
    }
    if (hcan == &hcan2){
        //省略
    }
        //省略部分代码
        return (tmp_tx_data_ptr);//返回值为指针存放地址
}
```
【说明】😎😎😎

原理：根据传入的CAN 外设句柄（区分hcan1/hcan2）和大疆电机 ID（枚举类型），返回对应的CAN 发送数据缓冲区的指针（uint8_t*）。

背景：大疆电机（如 C620、GM6020、M3508 等）的 CAN 控制协议采用“单帧多电机”的高效设计：一个 8 字节的 CAN 发送帧（CAN 数据帧最大长度为 8 字节）可以拆分为4 个 2 字节的控制段，分别对应 4 个不同 ID 的电机（每个电机的控制指令为 2 字节，如电流指令）。

另外，在drv_can.cpp文件中定义了
```c
    Struct_CAN_Manage_Object CAN1_Manage_Object = {0};
    Struct_CAN_Manage_Object CAN2_Manage_Object = {0};
```
并且在drv_can.h里面有
```c
struct Struct_CAN_Manage_Object
{
    CAN_HandleTypeDef *CAN_Handler;
    Struct_CAN_Rx_Buffer Rx_Buffer;
    CAN_Call_Back Callback_Function;
};
```c
其中Struct_CAN_Rx_Buffer也在drv_can.h

/**
 * @brief CAN接收的信息结构体
 *
 */
struct Struct_CAN_Rx_Buffer
{
    CAN_RxHeaderTypeDef Header;
    uint8_t Data[8];
};
```

另另外在drv_can.h里面还有

```c
//绑定的CAN
    Struct_CAN_Manage_Object *CAN_Manage_Object;
```
### 2.数据处理
    void Class_DJI_Motor_C620::Data_Process()
【功能】解析电机通过 CAN 反馈的原始数据，计算电机的角度、角速度、扭矩等物理量，并处理编码器圈数累计。
### 🤣😂🤣难点
Q：处理编码器圈数这是不是跟时间有关？剩下的一圈会在下一次数据处理时继续判定？🧐

A：哪怕极端情况下电机单次间隔内转了 2 圈，后续的 CAN 数据会通过新的编码器差值，再次触发跨圈判断，最终Total_Round会修正为实际的圈数。

1.**编码器数值持续更新**：电机的编码器值会随着转动不断变化，下一次收到的tmp_encoder会基于本次的tmp_encoder继续变化，新的差值会反映之前未被判定的圈数。

2.**代码逐次修正圈数**：代码的逻辑是每次数据处理只修正一次圈数（±1），但会通过多次数据处理，逐步累加总圈数，最终覆盖电机实际转动的圈数。

3.**工程场景的合理性**：CAN 通信周期极短，电机单次间隔内转多圈的情况几乎不存在，因此 “剩余圈数在下一次判定” 是理论上的补充逻辑，实际中几乎不会触发，但代码的设计仍能兼容这种极端情况。
### 3.CAN通信接收回调函数(Rx_Data 接收的数据)
```c
void Class_DJI_Motor_C620::CAN_RxCpltCallback(uint8_t *Rx_Data)
{
    //滑动窗口, 判断电机是否在线
    Flag += 1;

    Data_Process();
}
```
函数2在函数3里面，函数3里面的Flag又与函数4有关
### 4.TIM定时器中断定期检测电机是否存活
    void Class_DJI_Motor_C620::TIM_Alive_PeriodElapsedCallback()
### 5.TIM定时器中断计算回调函数
先判断电机控制模式，最终输出out
```c
void Class_DJI_Motor_C620::TIM_PID_PeriodElapsedCallback()
{
    switch (DJI_Motor_Control_Method)
    {
    case (DJI_Motor_Control_Method_OPENLOOP):
    {
        //默认开环扭矩控制
        Out = Target_Torque / Torque_Max * Output_Max;
    }
    break;
    case (DJI_Motor_Control_Method_TORQUE):
    {
        //默认闭环扭矩控制
        Out = Target_Torque / Torque_Max * Output_Max;
    }
    break;
    case (DJI_Motor_Control_Method_OMEGA):
    {
        PID_Omega.Set_Target(Target_Omega_Radian);
        PID_Omega.Set_Now(Data.Now_Omega_Radian);
        PID_Omega.TIM_Adjust_PeriodElapsedCallback();

        Out = PID_Omega.Get_Out();
    }
    break;
    case (DJI_Motor_Control_Method_ANGLE):
    {
        PID_Angle.Set_Target(Target_Radian);
        PID_Angle.Set_Now(Data.Now_Radian);
        PID_Angle.TIM_Adjust_PeriodElapsedCallback();

        Target_Omega_Radian = PID_Angle.Get_Out();

        PID_Omega.Set_Target(Target_Omega_Radian);
        PID_Omega.Set_Now(Data.Now_Omega_Radian);
        PID_Omega.TIM_Adjust_PeriodElapsedCallback();

        Out = PID_Omega.Get_Out();
    }
    break;
    default:
    {
        Out = 0.0f;
    }
    break;
    }
    Output();
}
```
函数5调用后也包含了函数6，将PID输出的Out值赋给了发送缓存区。
而且Class_Tricycle_Chassis::Speed_Resolution()中调用了这个函数！

✌️✌️✌️*！！！PID与CAN的连接*
### 6.电机数据输出到CAN总线发送缓冲区
```c
void Class_DJI_Motor_C620::Output()
{
    CAN_Tx_Data[0] = (int16_t)Out >> 8;
    CAN_Tx_Data[1] = (int16_t)Out;
}
```
😊😊😊前面已经提到 分配发送缓存区的地址给CAN_Tx_Data，那么接下来给这个地址下面的[0][1]赋值PID输出值，其实就是给发送缓存区对应位置赋值。

### 【总结】：一定要自己再补代码的是函数1和3，函数5似乎是可选项。
还有个问题，那么Out的值从哪儿来呢？ ------看函数5。
### 【summary】
1、初始化：绑定 CAN 总线、配置参数、分配缓冲区。

2、数据接收：CAN 中断接收电机反馈数据，标记在线状态。

3、数据处理：解析原始数据，计算角度、角速度等物理量。

4、PID 控制：定时器中断执行 PID 算法，根据控制方式输出指令。

5、数据发送：将控制指令拆分为字节，存入 CAN 发送缓冲区。

6、在线检测：定时器定期检测电机是否在线，处理离线保护逻辑。