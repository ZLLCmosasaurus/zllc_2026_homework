#include "task.h"
#include "drv_can.h"
#include "drv_tim.h"
#include "dvc_djimotor.h"

Class_DJI_Motor_C620 motor_3508;

void Chassis_Device_CAN1_Callback(Struct_CAN_Rx_Buffer *CAN_RxMessage)
{
    switch (CAN_RxMessage->Header.StdId)
    {
        case (0x201):
        {
            //chariot.Chassis.Motor_Wheel[0].CAN_RxCpltCallback(CAN_RxMessage->Data);
            motor_3508.CAN_RxCpltCallback(CAN_RxMessage->Data);
        }
        break;
        // case (0x202):
        // {
        //     chariot.Chassis.Motor_Wheel[1].CAN_RxCpltCallback(CAN_RxMessage->Data);
        // }
        // break;
        // case (0x203):
        // {
        //     chariot.Chassis.Motor_Wheel[2].CAN_RxCpltCallback(CAN_RxMessage->Data);
        // }
        // break;
        // case (0x204):
        // {
        //     chariot.Chassis.Motor_Wheel[3].CAN_RxCpltCallback(CAN_RxMessage->Data);
        // }
        // break;
        // case (0x206):  
        // {
            
        // }
        // break;
        // case (0x207):
        // {
            
        // }
        // break;
    }
}

uint32_t init_finished =0 ;
bool start_flag=0;
void Task1ms_TIM5_Callback()
{   //给外设留初始化的时间
    init_finished++;
    if(init_finished>2000)
    start_flag=1;
    if(start_flag==1){
       // chariot.TIM_Calculate_PeriodElapsedCallback();

        motor_3508.TIM_PID_PeriodElapsedCallback();
        TIM_CAN_PeriodElapsedCallback();

     //   TIM_UART_PeriodElapsedCallback();
    }


}


  void Task_Init()
  {
     //can
     CAN_Init(&hcan1, Chassis_Device_CAN1_Callback);
     //tim
     TIM_Init(&htim5,Task1ms_TIM5_Callback);
     HAL_TIM_Base_Start_IT(&htim5);
     //motor3508
     motor_3508.Init(&hcan1,DJI_Motor_ID_0x201,DJI_Motor_Control_Method_OMEGA,19.2f);
     motor_3508.PID_Omega.Init(1.0f,0.0f,0.0f,0.0f);
  }

  void Task_Loop()
  {

  }