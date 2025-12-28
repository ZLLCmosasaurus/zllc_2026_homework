#include "task.h"
#include "drv_can.h"
#include "drv_tim.h"
#include "drv_uart.h"

#include "dvc_djimotor.h"
#include "dvc_dr16.h"



#include "crt_chassis.h"
#include "dvc_referee.h"
#include "alg_fsm.h"
Class_DJI_Motor_C620 motor_3508; 
Class_DJI_Motor_C620 motor_wheel[5];
Class_DR16 DR16;
Class_Referee referee;
Struct_Referee_Rx_Data_Robot_Status referee_robot_status;
Struct_Referee_Rx_Data_Robot_Power_Heat referee_robot_power_heat;
int flag=0;

Class_Tricycle_Chassis chassis;

//float v_x_max,v_y_max;

//class Class_FSM_Alive_Control : public Class_FSM
//{
//public:
//    Class_Chariot *Chariot;
//    
//    // 新增：底盘控制类型缓存，用于状态切换时的恢复
//    Enum_Chassis_Control_Type chassis_control_cache;
//    
//    // 新增：遥控器在线状态标志
//    bool is_dr16_online;

//    void Reload_TIM_Status_PeriodElapsedCallback();
//    
//    // 新增：底盘控制管理函数
//    void Manage_Chassis_Control();
//};

//uint8_t Pre_Chassis_Control_Type,Now_Chassis_Control_Type;

//void Class_FSM_Alive_Control::Reload_TIM_Status_PeriodElapsedCallback()
//{
//    Status[Now_Status_Serial].Time++;

//    switch (Now_Status_Serial)
//    {
//        // 离线检测状态
//        case (0):
//        {
//            // 遥控器中途断联导致错误离线 跳转到 遥控器串口错误状态
//            if (huart3.ErrorCode)
//            {
//                Status[Now_Status_Serial].Time = 0;
//                Set_Status(4);
//                return;
//            }

//            // 转移为 在线状态
//            if(DR16.Get_DR16_Status() == DR16_Status_ENABLE)
//            {             
//                Status[Now_Status_Serial].Time = 0;
//                is_dr16_online = true;  // 设置在线标志
//                Set_Status(2);
//                return;
//            }

//            // 超过一秒的遥控器离线 跳转到 遥控器关闭状态
//            if(Status[Now_Status_Serial].Time > 1000)
//            {
//                Status[Now_Status_Serial].Time = 0;
//                is_dr16_online = false;  // 设置离线标志
//                Set_Status(1);
//                return;
//            }
//        }
//        break;
//        
//        // 遥控器关闭状态
//        case (1):
//        {
//            // 离线时，记录当前底盘控制模式并禁用底盘
//            if (is_dr16_online)
//            {
//                // 如果遥控器重新在线，恢复到之前的控制模式
//                chassis.Set_Chassis_Control_Type(chassis_control_cache);
//                is_dr16_online = true;
//                Status[Now_Status_Serial].Time = 0;
//                Set_Status(2);
//                return;
//            }
//            
//            // 持续离线，保持底盘禁用
//            chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE);
//            
//            // 检查是否恢复在线
//            if(DR16.Get_DR16_Status() == DR16_Status_ENABLE)
//            {
//                is_dr16_online = true;
//                chassis.Set_Chassis_Control_Type(chassis_control_cache);
//                Status[Now_Status_Serial].Time = 0;
//                Set_Status(2);
//                return;
//            }

//            // 遥控器中途断联导致错误离线 跳转到 遥控器串口错误状态
//            if (huart3.ErrorCode)
//            {
//                Status[Now_Status_Serial].Time = 0;
//                Set_Status(4);
//                return;
//            }
//        }
//        break;
//        
//        // 遥控器在线状态
//        case (2):
//        {
//            // 检查遥控器是否离线
//            if(DR16.Get_DR16_Status() == DR16_Status_DISABLE)
//            {
//                // 离线时缓存当前控制模式
//                chassis_control_cache = (Enum_Chassis_Control_Type)chassis.Get_Chassis_Control_Type();
//                is_dr16_online = false;
//                Status[Now_Status_Serial].Time = 0;
//                Set_Status(3);
//                return;
//            }
//            
//            // 遥控器在线，确保底盘控制模式不是DISABLE
//            if (chassis.Get_Chassis_Control_Type() == Chassis_Control_Type_DISABLE)
//            {
//                // 如果是DISABLE，恢复到缓存的模式
//                chassis.Set_Chassis_Control_Type(chassis_control_cache);
//            }
//        }
//        break;
//        
//        // 刚离线状态 - 过渡状态
//        case (3):
//        {
//            // 立即转移到离线检测状态
//            Status[Now_Status_Serial].Time = 0;
//            Set_Status(0);
//        }
//        break;
//        
//        // 遥控器串口错误状态
//        case (4):
//        {
//            HAL_UART_DMAStop(&huart3); // 停止以重启
//            HAL_UARTEx_ReceiveToIdle_DMA(&huart3, UART3_Manage_Object.Rx_Buffer, UART3_Manage_Object.Rx_Buffer_Length);

//            // 处理完跳转到离线检测状态
//            Status[Now_Status_Serial].Time = 0;
//            is_dr16_online = false;
//            Set_Status(0);
//        }
//        break;
//    } 
//}

//void Class_FSM_Alive_Control::Manage_Chassis_Control()
//{
//    // 只在遥控器在线时处理底盘控制
//    if (is_dr16_online && DR16.Get_DR16_Status() == DR16_Status_ENABLE)
//    {
//        // 如果当前是禁用状态，恢复到缓存的模式
//        if (chassis.Get_Chassis_Control_Type() == Chassis_Control_Type_DISABLE)
//        {
//            chassis.Set_Chassis_Control_Type(chassis_control_cache);
//        }
//    }
//}

//Class_FSM_Alive_Control FSM_Controller;

//void Chassis_Device_CAN1_Callback(Struct_CAN_Rx_Buffer *CAN_RxMessage)
//{
//    switch (CAN_RxMessage->Header.StdId)
//    {
//        case (0x201):
//        {
//            chassis.Motor_Wheel[0].CAN_RxCpltCallback(CAN_RxMessage->Data);
//        }
//        break;
//        case (0x202):
//        {
//            chassis.Motor_Wheel[1].CAN_RxCpltCallback(CAN_RxMessage->Data);
//        }
//        break;
//        case (0x203):
//        {
//            chassis.Motor_Wheel[2].CAN_RxCpltCallback(CAN_RxMessage->Data);
//        }
//        break;
//        case (0x204):
//        {
//            chassis.Motor_Wheel[3].CAN_RxCpltCallback(CAN_RxMessage->Data);
//        }
//        break;
//        case (0x206):  
//        {
//            
//        }
//        break;
//        case (0x207):
//        {
//            
//        }
//        break;
//    }
//}

//void DR16_UART3_Callback(uint8_t *Buffer, uint16_t Length)
//{
//    DR16.DR16_UART_RxCpltCallback(Buffer);
//    // 遥控器数据到达，确保状态机知道遥控器在线
//    if (!FSM_Controller.is_dr16_online)
//    {
//        FSM_Controller.is_dr16_online = true;
//    }
//}

//void Image_UART1_Callback(uint8_t *Buffer, uint16_t Length)
//{
//    DR16.Image_UART_RxCpltCallback(Buffer);
//}

void Referee_UART6_Callback(uint8_t *Buffer, uint16_t Length)
{ // 空实现：dvc_referee.cpp内部已绑定UART解析，无需外部调用
    // 仅重启UART接收，确保持续接收数据
    HAL_UART_Receive_IT(&huart6, UART6_Manage_Object.Rx_Buffer, UART6_Manage_Object.Rx_Buffer_Length);
    referee.UART_RxCpltCallback(Buffer, Length);
}
void Task1ms_TIM5_Callback()
{
    // 1. 获取遥控器输入
//    if (FSM_Controller.is_dr16_online && DR16.Get_DR16_Status() == DR16_Status_ENABLE)
//    {
//        // 只在遥控器在线时接受遥控器控制
//        chassis.Set_Target_Velocity_X(DR16.Get_Left_X() * chassis.Get_Velocity_X_Max());
//        chassis.Set_Target_Velocity_Y(DR16.Get_Left_Y() * chassis.Get_Velocity_Y_Max());
//        chassis.Set_Target_Omega(-1.0f * DR16.Get_Right_X() * chassis.Get_Omega_Max());
//    }
//    else
//    {
//        // 遥控器离线时，停止底盘
//        chassis.Set_Target_Velocity_X(0);
//        chassis.Set_Target_Velocity_Y(0);
//        chassis.Set_Target_Omega(0);
//    }
//    
//    // 2. 执行底盘计算
//    chassis.TIM_Calculate_PeriodElapsedCallback(Sprint_Status_DISABLE);
//    
//    // 3. 状态机更新
//    FSM_Controller.Reload_TIM_Status_PeriodElapsedCallback();
//    
//    // 4. 底盘控制管理（确保在线时不是禁用状态）
//    FSM_Controller.Manage_Chassis_Control();
    
  // 5. 裁判系统数据更新与解包
    static uint32_t referee_update_counter = 0;
    referee_update_counter++;
    if (referee_update_counter >= 50) // 50ms更新一次（20Hz）
    {
        referee_update_counter = 0;
        
        // 更新裁判系统存活检测
        referee.TIM1msMod50_Alive_PeriodElapsedCallback();
        
        // 获取机器人状态数据（如果裁判系统在线）
        if (referee.Get_Referee_Status() == Referee_Status_ENABLE)
        {
           // 解包 Struct_Referee_Rx_Data_Robot_Status
            referee_robot_status.Robot_ID = referee.Get_ID();
            referee_robot_status.Level = referee.Get_Level();
            referee_robot_status.HP = referee.Get_HP();
            referee_robot_status.HP_Max = referee.Get_HP_Max();
            referee_robot_status.Shooter_Barrel_Cooling_Value = referee.Get_Booster_17mm_1_Heat_CD();
            referee_robot_status.Shooter_Barrel_Heat_Limit = referee.Get_Booster_17mm_1_Heat_Max();
            referee_robot_status.Chassis_Power_Limit = referee.Get_Chassis_Power_Max();
            
            // 获取供电状态
            referee_robot_status.PM01_Gimbal_Status_Enum = referee.Get_PM01_Gimbal_Status();
            referee_robot_status.PM01_Chassis_Status_Enum = referee.Get_PM01_Chassis_Status();
            referee_robot_status.PM01_Booster_Status_Enum = referee.Get_PM01_Booster_Status();
            
            // 解包 Struct_Referee_Rx_Data_Robot_Power_Heat（对应 Referee_Command_ID_ROBOT_POWER_HEAT）
            referee_robot_power_heat.Chassis_Voltage = referee.Get_Chassis_Voltage() * 1000; // 转换为mV
            referee_robot_power_heat.Chassis_Current = referee.Get_Chassis_Current() * 1000; // 转换为mA
            referee_robot_power_heat.Chassis_Power = referee.Get_Chassis_Power();
            referee_robot_power_heat.Chassis_Energy_Buffer = referee.Get_Chassis_Energy_Buffer();
            referee_robot_power_heat.Booster_17mm_1_Heat = referee.Get_Booster_17mm_1_Heat();
            referee_robot_power_heat.Booster_17mm_2_Heat = referee.Get_Booster_17mm_2_Heat();
            referee_robot_power_heat.Booster_42mm_Heat = referee.Get_Booster_42mm_Heat();
    }
    
    // 6. 其他定时任务
    TIM_CAN_PeriodElapsedCallback();
    TIM_UART_PeriodElapsedCallback();
}
		}

void Task_Init()
{
//    CAN_Init(&hcan1, Chassis_Device_CAN1_Callback);
//    UART_Init(&huart3, DR16_UART3_Callback, 18);
////    UART_Init(&huart1, Image_UART1_Callback, 40);
//    TIM_Init(&htim5, Task1ms_TIM5_Callback);
    UART_Init(&huart6, Referee_UART6_Callback, 64);
 referee.Init(&huart6, 0xA5);
    // 底盘初始化
//    chassis.Init(4.0f, 4.0f, 8.0f, 0.5f);
//    
//    // 设置初始控制模式
//    chassis.Set_Chassis_Control_Type(Chassis_Control_Type_FLLOW);
//    
//    // 初始化状态机
//    FSM_Controller.Init(5);
//    FSM_Controller.chassis_control_cache = Chassis_Control_Type_FLLOW;  // 缓存初始模式
//    FSM_Controller.is_dr16_online = false;  // 初始状态为离线
//    
//    // 遥控器初始化
//    DR16.Init(&huart3, &huart1);
    
    // 启动定时器
    HAL_TIM_Base_Start_IT(&htim5);
    
//    // 获取速度最大值
//    v_x_max = chassis.Get_Velocity_X_Max();
//    v_y_max = chassis.Get_Velocity_Y_Max();
}
void Task_Loop()
{
   
}
