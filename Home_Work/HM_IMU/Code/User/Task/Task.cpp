#include "Task.h"
#include "alg_fsm.h"
#include "crt_chassis.h"
#include "drv_can.h"
#include "drv_tim.h"
#include "drv_uart.h"
#include "dvc_djimotor.h"
#include "dvc_dr16.h"
#include "dvc_imu.h"

#define REDUCTION 3591.0f / 187.0f // 减速比

// Class_DJI_Motor_C620 Motor_Wheel[4];
Class_Tricycle_Chassis Chassis;
Class_DR16 DR16;
Class_FSM FSM_Alive_Control;
Class_IMU IMU;

uint32_t init_finished = 0;
uint32_t imu_counter = 0;
bool start_flag = 0;

void Chassis_Device_CAN1_Callback(Struct_CAN_Rx_Buffer *CAN_RxMessage) {
    switch (CAN_RxMessage->Header.StdId) {
    case (0x201): {
        Chassis.Motor_Wheel[0].CAN_RxCpltCallback(CAN_RxMessage->Data);
    } break;
    case (0x202): {
        Chassis.Motor_Wheel[1].CAN_RxCpltCallback(CAN_RxMessage->Data);
    } break;
    case (0x203): {
        Chassis.Motor_Wheel[2].CAN_RxCpltCallback(CAN_RxMessage->Data);
    } break;
    case (0x204): {
        Chassis.Motor_Wheel[3].CAN_RxCpltCallback(CAN_RxMessage->Data);
    } break;
    }
}

void Task1ms_TIM5_Callback() {
    // 50秒执行一次IMU状态检测
    if (imu_counter >= 50) {
        imu_counter = 0;
        IMU.TIM1msMod50_Alive_PeriodElapsedCallback();
    }
    IMU.TIM_Calculate_PeriodElapsedCallback();

    init_finished++;
    imu_counter++;
    if (init_finished > 2000) // 等初始化完成后再启动底盘
        start_flag = 1;

    if (start_flag == 1) {
        FSM_Alive_Control.Reload_TIM_Status_PeriodElapsedCallback();

        Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_FLLOW);
        Chassis.Set_Target_Velocity_X(DR16.Get_Left_X());
        Chassis.Set_Target_Velocity_Y(DR16.Get_Left_Y());
        Chassis.Set_Target_Omega(-DR16.Get_Right_X());
        Chassis.TIM_Calculate_PeriodElapsedCallback(Sprint_Status_ENABLE);

        TIM_CAN_PeriodElapsedCallback(); // 打包发送数据
    }
}

void DR16_UART3_Callback(uint8_t *Buffer, uint16_t Length) {

    DR16.DR16_UART_RxCpltCallback(Buffer);
}

void Task_Init() {
    // DWT初始化
    DWT_Init(168);
    // can总线
    CAN_Init(&hcan1, Chassis_Device_CAN1_Callback);
    // tim
    TIM_Init(&htim5, Task1ms_TIM5_Callback);
    HAL_TIM_Base_Start_IT(&htim5);
    TIM_Init(&htim10, NULL);
    HAL_TIM_Base_Start_IT(&htim10);
    // 串口
    UART_Init(&huart3, DR16_UART3_Callback, 18);
    // 初始化底盘
    Chassis.Init();
    // DR16初始化
    DR16.Init(&huart3, &huart6);
    // SPI初始化
    SPI_Init(&hspi1, NULL);
    // IMU初始化
    IMU.Init();
}

void Task_Loop() {
}
