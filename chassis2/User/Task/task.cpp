#include "task.h"
#include "drv_can.h"
#include "drv_tim.h"
#include "dvc_djimotor.h"
#include "crt_chassis.h"
#include "dvc_dr16.h"
Class_Tricycle_Chassis chassis;
Class_DR16 DR16;

void Chassis_Device_CAN1_Callback(Struct_CAN_Rx_Buffer *CAN_RxMessage)//接收电调数据
{
    switch (CAN_RxMessage->Header.StdId)
    {
        case (0x201):
        {
           // chariot.Chassis.Motor_Wheel[0].CAN_RxCpltCallback(CAN_RxMessage->Data);
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
       
    }
}


void DR16_UART3_Callback(uint8_t *Buffer, uint16_t Length)
{

    DR16.DR16_UART_RxCpltCallback(Buffer);

   
}


void Task1ms_TIM5_Callback()
{
    chassis.TIM_Calculate_PeriodElapsedCallback(Sprint_Status_DISABLE);
    TIM_CAN_PeriodElapsedCallback();
    static uint8_t count=0;
    if(++count>=50)
    {
        count=0;
        DR16.TIM1msMod50_Alive_PeriodElapsedCallback();
    }
}

void Task_Init()
{
    
//can
CAN_Init(&hcan1,Chassis_Device_CAN1_Callback);
//tim
TIM_Init(&htim5,Task1ms_TIM5_Callback);
 //chassis
chassis.Init();
chassis.Set_Chassis_Control_Type(Chassis_Control_Type_FLLOW);
//dr16
DR16.Init(&huart3,nullptr);
UART_Init(&huart3, DR16_UART3_Callback, 18);
}
void Task_Loop(){
float target_x=0.0f;
float target_y=0.0f;
float target_omega=0.0f;


if(DR16.Get_DR16_Status()==DR16_Status_ENABLE)
  {
target_x= DR16.Get_Right_X()*4.0f;
target_y= DR16.Get_Right_Y()*4.0f;
target_omega= DR16.Get_Left_X()*8.0f;
}
else
{
target_x=0.0f;
target_y=0.0f;
target_omega=0.0f;
}


chassis.Set_Target_Velocity_X(target_x);
chassis.Set_Target_Velocity_Y(target_y);
chassis.Set_Target_Omega(target_omega);

  }
 