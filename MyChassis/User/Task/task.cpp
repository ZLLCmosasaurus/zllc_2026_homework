#include "task.h"
#include "drv_can.h"
#include "drv_tim.h"
#include "drv_uart.h"

#include "dvc_djimotor.h"
#include "dvc_dr16.h"

#include "crt_chassis.h"

#include "alg_fsm.h"

Class_DJI_Motor_C620 motor_3508; 
Class_DJI_Motor_C620 motor_wheel[5];
Class_DR16 DR16;

int flag=0;


Class_Tricycle_Chassis chassis;

float v_x_max,v_y_max;

class Class_FSM_Alive_Control : public Class_FSM
{
public:
    Class_Chariot *Chariot;

    void Reload_TIM_Status_PeriodElapsedCallback();
};

uint8_t Pre_Chassis_Control_Type,Now_Chassis_Control_Type;

void Class_FSM_Alive_Control::Reload_TIM_Status_PeriodElapsedCallback()
{
    Status[Now_Status_Serial].Time++;

    switch (Now_Status_Serial)
    {
        // 离线检测状态
        case (0):
        {
            // 遥控器中途断联导致错误离线 跳转到 遥控器串口错误状态
            if (huart3.ErrorCode)
            {
                Status[Now_Status_Serial].Time = 0;
                Set_Status(4);
            }

            //转移为 在线状态
            if(DR16.Get_DR16_Status() == DR16_Status_ENABLE)
            {             
                Status[Now_Status_Serial].Time = 0;
                Set_Status(2);
            }

            //超过一秒的遥控器离线 跳转到 遥控器关闭状态
            if(Status[Now_Status_Serial].Time > 1000)
            {
                Status[Now_Status_Serial].Time = 0;
                Set_Status(1);
            }
        }
        break;
        // 遥控器关闭状态
        case (1):
        {
            //离线保护
            //Chariot->Booster.Set_Booster_Control_Type(Booster_Control_Type_DISABLE);
            //Chariot->Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_DISABLE);
            chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE);
            Now_Chassis_Control_Type = Chassis_Control_Type_DISABLE;
            if(DR16.Get_DR16_Status() == DR16_Status_ENABLE)
            {
                chassis.Set_Chassis_Control_Type((Enum_Chassis_Control_Type)Pre_Chassis_Control_Type);
                //Chariot->Gimbal.Set_Gimbal_Control_Type(Chariot->Get_Pre_Gimbal_Control_Type());
                Status[Now_Status_Serial].Time = 0;
                Set_Status(2);
            }

            // 遥控器中途断联导致错误离线 跳转到 遥控器串口错误状态
            if (huart3.ErrorCode)
            {
                Status[Now_Status_Serial].Time = 0;
                Set_Status(4);
            }
            
        }
        break;
        // 遥控器在线状态
        case (2):
        {
            //转移为 刚离线状态
            if(DR16.Get_DR16_Status() == DR16_Status_DISABLE)
            {
                Status[Now_Status_Serial].Time = 0;
                Set_Status(3);
            }
        }
        break;
        //刚离线状态
        case (3):
        {
            //记录离线检测前控制模式
            Pre_Chassis_Control_Type = chassis.Get_Chassis_Control_Type();
            //Chariot->Set_Pre_Chassis_Control_Type(Chariot->Chassis.Get_Chassis_Control_Type());
            //Chariot->Set_Pre_Gimbal_Control_Type(Chariot->Gimbal.Get_Gimbal_Control_Type());

            //无条件转移到 离线检测状态
            Status[Now_Status_Serial].Time = 0;
            Set_Status(0);
        }
        break;
        //遥控器串口错误状态
        case (4):
        {
            HAL_UART_DMAStop(&huart3); // 停止以重启
            //HAL_Delay(10); // 等待错误结束
            HAL_UARTEx_ReceiveToIdle_DMA(&huart3, UART3_Manage_Object.Rx_Buffer, UART3_Manage_Object.Rx_Buffer_Length);

            //处理完直接跳转到 离线检测状态
            Status[Now_Status_Serial].Time = 0;
            Set_Status(0);
        }
        break;
    } 
}
Class_FSM_Alive_Control FSM_Controller;

void Chassis_Device_CAN1_Callback(Struct_CAN_Rx_Buffer *CAN_RxMessage)
{
    switch (CAN_RxMessage->Header.StdId)
    {
        case (0x201):
        {
            chassis.Motor_Wheel[0].CAN_RxCpltCallback(CAN_RxMessage->Data);
        }
        break;
        case (0x202):
        {
            chassis.Motor_Wheel[1].CAN_RxCpltCallback(CAN_RxMessage->Data);
        }
        break;
        case (0x203):
        {
            chassis.Motor_Wheel[2].CAN_RxCpltCallback(CAN_RxMessage->Data);
        }
        break;
        case (0x204):
        {
            chassis.Motor_Wheel[3].CAN_RxCpltCallback(CAN_RxMessage->Data);
        }
        break;
        case (0x206):  
        {
            
        }
        break;
        case (0x207):
        {
            
        }
        break;
    }
}

void DR16_UART3_Callback(uint8_t *Buffer, uint16_t Length)
{
    DR16.DR16_UART_RxCpltCallback(Buffer);
    //Judge_DR16_Control_Type();
}

void Image_UART1_Callback(uint8_t *Buffer, uint16_t Length)
{
    DR16.Image_UART_RxCpltCallback(Buffer);
}

void Task1ms_TIM5_Callback()
{
    //flag++;
    //motor_3508.TIM_PID_PeriodElapsedCallback();
    chassis.Set_Target_Velocity_X(DR16.Get_Left_X()*chassis.Get_Velocity_X_Max());
    chassis.Set_Target_Velocity_Y(DR16.Get_Left_Y()*chassis.Get_Velocity_Y_Max());
    chassis.Set_Target_Omega(-1.0*DR16.Get_Right_X()*chassis.Get_Omega_Max());
    // chassis.Set_Target_Velocity_X(1.0f);
    // chassis.Set_Target_Velocity_Y(0.0f);
    //chassis.Set_Now_Velocity_X(1.0f);
    //chassis.Set_Now_Velocity_Y(0.0f);
    chassis.TIM_Calculate_PeriodElapsedCallback(Sprint_Status_DISABLE);
    //DR16.TIM1msMod50_Alive_PeriodElapsedCallback();
    FSM_Controller.Reload_TIM_Status_PeriodElapsedCallback();
    TIM_CAN_PeriodElapsedCallback();
    TIM_UART_PeriodElapsedCallback();
}

void Task_Init()
{
    CAN_Init(&hcan1, Chassis_Device_CAN1_Callback);
    UART_Init(&huart3, DR16_UART3_Callback, 18);
    UART_Init(&huart1, Image_UART1_Callback, 40);
    TIM_Init(&htim5,Task1ms_TIM5_Callback);
    
    chassis.Init(4.0f,4.0f,8.0f,0.5f);
    chassis.Set_Chassis_Control_Type(Chassis_Control_Type_FLLOW);
    Pre_Chassis_Control_Type = Chassis_Control_Type_FLLOW;
    // //motor_3508.Init(&hcan1,DJI_Motor_ID_0x201,DJI_Motor_Control_Method_OMEGA,19.0f);
    DR16.Init(&huart3,&huart1);
    HAL_TIM_Base_Start_IT(&htim5);
    // //motor_3508.PID_Omega.Init(1.0f,1.0f,0.0f,1.0f);
    FSM_Controller.Init(5);
    v_x_max=chassis.Get_Velocity_X_Max();
    v_y_max=chassis.Get_Velocity_Y_Max();
}

void Task_Loop()
{

}

