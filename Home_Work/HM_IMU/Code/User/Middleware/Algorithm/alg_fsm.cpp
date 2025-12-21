/**
 * @file alg_fsm.cpp
 * @author lez by yssickjgd
 * @brief 有限自动机
 * @version 0.1
 * @date 2024-07-1 0.1 24赛季定稿
 *
 * @copyright ZLLC 2024
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "alg_fsm.h"
#include "usart.h"
#include "dvc_dr16.h"
#include "crt_chassis.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
extern Class_DR16 DR16;
extern Class_Tricycle_Chassis Chassis;
/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 状态机初始化
 *
 * @param __Status_Number 状态数量
 * @param __Now_Status_Serial 当前指定状态机初始编号
 */
void Class_FSM::Init(uint8_t __Status_Number, uint8_t __Now_Status_Serial) {
    Status_Number = __Status_Number;

    Now_Status_Serial = __Now_Status_Serial;

    // 所有状态全刷0
    for (int i = 0; i < Status_Number; i++) {
        Status[i].Status_Stage = Status_Stage_DISABLE;
        Status[i].Time = 0;
    }

    // 使能初始状态
    Status[__Now_Status_Serial].Status_Stage = Status_Stage_ENABLE;
}

/**
 * @brief 定时器处理函数
 * 这是一个模板, 使用时请根据不同处理情况在不同文件内重新定义
 *
 */
void Class_FSM::Reload_TIM_Status_PeriodElapsedCallback() {
    Status[Now_Status_Serial].Time++;
    ///*
            //自己接着编写状态转移函数
            switch (Now_Status_Serial) {
            // 离线检测状态
            case (0): {
                // 遥控器中途断联导致错误离线 跳转到 遥控器串口错误状态
                if (huart3.ErrorCode) {
                    Status[Now_Status_Serial].Time = 0;
                    Set_Status(4);
                }

                // 转移为 在线状态
                if (DR16.Get_DR16_Status() == DR16_Status_ENABLE) {
                    Status[Now_Status_Serial].Time = 0;
                    Set_Status(2);
                }

                // 超过一秒的遥控器离线 跳转到 遥控器关闭状态
                if (Status[Now_Status_Serial].Time > 1000) {
                    Status[Now_Status_Serial].Time = 0;
                    Set_Status(1);
                }
            } break;
            // 遥控器关闭状态
            case (1): {
                // 离线保护
                Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE);

                if (DR16.Get_DR16_Status() == DR16_Status_ENABLE) {
                    Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_FLLOW);
                    Status[Now_Status_Serial].Time = 0;
                    Set_Status(2);
                }

                // 遥控器中途断联导致错误离线 跳转到 遥控器串口错误状态
                if (huart3.ErrorCode) {
                    Status[Now_Status_Serial].Time = 0;
                    Set_Status(4);
                }

            } break;
            // 遥控器在线状态
            case (2): {
                // 转移为 刚离线状态
                if (DR16.Get_DR16_Status() == DR16_Status_DISABLE) {
                    Status[Now_Status_Serial].Time = 0;
                    Set_Status(3);
                }
            } break;
            // 刚离线状态
            case (3): {
                // // 记录离线检测前控制模式
                // Chariot->Set_Pre_Chassis_Control_Type(Chariot->Chassis.Get_Chassis_Control_Type());
                // Chariot->Set_Pre_Gimbal_Control_Type(Chariot->Gimbal.Get_Gimbal_Control_Type());

                // 无条件转移到 离线检测状态
                Status[Now_Status_Serial].Time = 0;
                Set_Status(0);
            } break;
            // 遥控器串口错误状态
            case (4): {
                HAL_UART_DMAStop(&huart3); // 停止以重启
                // HAL_Delay(10); // 等待错误结束
                HAL_UARTEx_ReceiveToIdle_DMA(&huart3, UART3_Manage_Object.Rx_Buffer, UART3_Manage_Object.Rx_Buffer_Length);

                // 处理完直接跳转到 离线检测状态
                Status[Now_Status_Serial].Time = 0;
                Set_Status(0);
            } break;
            }
    //*/
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
