/**
 * @file tsk_config_and_callback.cpp
 * @author lez by yssickjgd
 * @brief 临时任务调度测试用函数, 后续用来存放个人定义的回调函数以及若干任务
 * @version 0.1
 * @date 2024-07-1 0.1 24赛季定稿
 * @copyright ZLLC 2024
 */

/**
 * @brief 注意, 每个类的对象分为专属对象Specialized, 同类可复用对象Reusable以及通用对象Generic
 *
 * 专属对象:
 * 单对单来独打独
 * 比如交互类的底盘对象, 只需要交互对象调用且全局只有一个, 这样看来, 底盘就是交互类的专属对象
 * 这种对象直接封装在上层类里面, 初始化在上层类里面, 调用在上层类里面
 *
 * 同类可复用对象:
 * 各调各的
 * 比如电机的对象, 底盘可以调用, 云台可以调用, 而两者调用的是不同的对象, 这种就是同类可复用对象
 * 电机的pid对象也算同类可复用对象, 它们都在底盘类里初始化
 * 这种对象直接封装在上层类里面, 初始化在最近的一个上层专属对象的类里面, 调用在上层类里面
 *
 * 通用对象:
 * 多个调用同一个
 * 比如裁判系统对象, 底盘类要调用它做功率控制, 发射机构要调用它做出膛速度与射击频率的控制, 因此裁判系统是通用对象.
 * 这种对象以指针形式进行指定, 初始化在包含所有调用它的上层的类里面, 调用在上层类里面
 *
 */

/**
 * @brief TIM开头的默认任务均1ms, 特殊任务需额外标记时间
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "tsk_config_and_callback.h"
#include "drv_tim.h"
#include "ita_chariot.h"
#include "config.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

Class_DJI_Motor_C620 Motor_Wheel[4];
Class_DR16 DR16;
Enum_Chassis_Control_Type Chassis_Control_Type = Chassis_Control_Type_DISABLE;
/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief Chassis_CAN1回调函数
 *
 * @param CAN_RxMessage CAN1收到的消息
 */
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

#ifdef LEARNING
void DR16_UART3_Callback(uint8_t *Buffer, uint16_t Length)
{
    DR16.DR16_UART_RxCpltCallback(Buffer);
}
#endif

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

/**
 * @brief TIM4任务回调函数
 *
 */
void Task100us_TIM4_Callback()
{
}

/**
 * @brief TIM5任务回调函数
 *
 */

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

/**
 * @brief 初始化任务
 *
 */
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

/**
 * @brief 前台循环任务
 *
 */
extern "C" void Task_Loop()
{
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
