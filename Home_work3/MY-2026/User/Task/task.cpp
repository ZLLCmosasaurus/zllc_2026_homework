#include "task.h"

#include "crt_chassis.h"

#include "drv_can.h"
#include "drv_tim.h"
#include "drv_uart.h"

#include "dvc_djimotor.h"
#include "dvc_dr16.h"

#include "alg_fsm.h"
#include "alg_pid.h"
Class_Tricycle_Chassis chassis;
//Class_DJI_Motor_C620 motor_3508;
//Class_DJI_Motor_C620 Motor_Wheel[4];
Class_DR16 DR16;


void Chassis_Device_CAN1_Callback(Struct_CAN_Rx_Buffer *CAN_RxMessage)
{
    switch (CAN_RxMessage->Header.StdId)
    {
        case (0x201):
        {
            //chariot.Chassis.Motor_Wheel[0].CAN_RxCpltCallback(CAN_RxMessage->Data);
            chassis.Motor_Wheel[0].CAN_RxCpltCallback(CAN_RxMessage->Data);
           // chassis.Motor_Wheel[0].TIM_PID_PeriodElapsedCallback();已有
           //chassis.Motor_Wheel[0].TIM_Alive_PeriodElapsedCallback();可选？
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

void DR16_UART3_Callback(uint8_t *Buffer, uint16_t Length)
{

    DR16.DR16_UART_RxCpltCallback(Buffer);

    //底盘 云台 发射机构 的控制策略
    //chariot.TIM_Control_Callback();
}

void Image_UART6_Callback(uint8_t *Buffer, uint16_t Length)
{
    DR16.Image_UART_RxCpltCallback(Buffer);

    //底盘 云台 发射机构 的控制策略
    //chariot.TIM_Control_Callback();
	
}

uint32_t init_finished =0 ;
bool start_flag=0;
void Task1ms_TIM5_Callback()
{   //给外设留初始化的时间
    init_finished++;
    if(init_finished>2000)
    start_flag=1;
    if(start_flag==1){

    chassis.Set_Target_Velocity_X(DR16.Get_Left_X()*chassis.Get_Velocity_X_Max());
    chassis.Set_Target_Velocity_Y(DR16.Get_Left_Y()*chassis.Get_Velocity_Y_Max());
    chassis.Set_Target_Omega(DR16.Get_Right_X()*chassis.Get_Omega_Max());
         TIM_CAN_PeriodElapsedCallback();

         TIM_UART_PeriodElapsedCallback();//不需要吧？

        //Motor_Wheel[0].TIM_PID_PeriodElapsedCallback();
         
    }


}


  void Task_Init()
  {
     //can
     CAN_Init(&hcan1, Chassis_Device_CAN1_Callback);
     //tim
     TIM_Init(&htim5,Task1ms_TIM5_Callback);
     HAL_TIM_Base_Start_IT(&htim5);
     //DR16
     DR16.Init(&huart3,&huart6);
     UART_Init(&huart3, DR16_UART3_Callback, 18);
     UART_Init(&huart6, Image_UART6_Callback, 40);
     //motor3508
    //  Motor_Wheel[0].Init(&hcan1,DJI_Motor_ID_0x201,DJI_Motor_Control_Method_OMEGA,19.2f);
    //  Motor_Wheel[1].Init(&hcan1,DJI_Motor_ID_0x201,DJI_Motor_Control_Method_OMEGA,19.2f);
    //  Motor_Wheel[2].Init(&hcan1,DJI_Motor_ID_0x201,DJI_Motor_Control_Method_OMEGA,19.2f);
    //  Motor_Wheel[3].Init(&hcan1,DJI_Motor_ID_0x201,DJI_Motor_Control_Method_OMEGA,19.2f);
     chassis.Init(4.0f,4.0f,8.0f,0.5f);
     chassis.Set_Chassis_Control_Type(Chassis_Control_Type_FLLOW);//Speed——Resolution里面要判断模式
     chassis.TIM_Calculate_PeriodElapsedCallback(Sprint_Status_ENABLE);//一会儿往上面1msTIM移，里面已经有内部函数void Speed_Resolution();
     //Motor_Wheel[0].PID_Omega.Init(1.0f,0.0f,0.0f,0.0f);
//lalala

  }

  void Task_Loop()
  {

  }