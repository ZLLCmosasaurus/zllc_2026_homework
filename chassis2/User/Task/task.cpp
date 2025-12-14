#include "task.h"
#include "drv_can.h"
#include "drv_tim.h"
#include "dvc_djimotor.h"
#include "crt_chassis.h"
#include "dvc_dr16.h"
#define GIMBAL
#define USE_DR16
#include "ita_chariot.h"
Class_Chariot chariot;
Class_Tricycle_Chassis chassis;
 

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

    chariot.DR16.DR16_UART_RxCpltCallback(Buffer);

   
}


void Task1ms_TIM5_Callback()
{
	static uint8_t count=0;
    if(++count>=10)
    {
        count=0;
        chariot.DR16.TIM1msMod50_Alive_PeriodElapsedCallback();
    }
     chassis.TIM_Calculate_PeriodElapsedCallback(Sprint_Status_DISABLE);
  
		 TIM_CAN_PeriodElapsedCallback();
	 
	
   //  same();
 chariot.FSM_Alive_Control.Reload_TIM_Status_PeriodElapsedCallback();
 chassis.Set_Chassis_Control_Type(chariot.Chassis.Get_Chassis_Control_Type());

    

     //DR16.TIM1msMod50_Alive_PeriodElapsedCallback();
    

   /* static uint8_t count=0;
    if(++count>=50)
    {
        count=0;
        DR16.TIM1msMod50_Alive_PeriodElapsedCallback();
    }*/
}

void Task_Init()
{
    
//can
CAN_Init(&hcan1,Chassis_Device_CAN1_Callback);
//tim
TIM_Init(&htim5,Task1ms_TIM5_Callback);
chariot.Init(0.1f);
 //chassis
chassis.Init();
chassis.Set_Chassis_Control_Type(Chassis_Control_Type_FLLOW);
//dr16
//chariot.DR16.Init(&huart3,nullptr);
UART_Init(&huart3, DR16_UART3_Callback, 18);
}
void Task_Loop(){
 

	if(chariot.DR16.Get_DR16_Status()==DR16_Status_DISABLE){
 
chassis.Set_Target_Velocity_X(0.0f);
chassis.Set_Target_Velocity_Y(0.0f);
chassis.Set_Target_Omega(0.0f);
	
 
	return;
	}
	else{
chassis.Set_Chassis_Control_Type(Chassis_Control_Type_FLLOW);
float target_x= chariot.DR16.Get_Right_X()*4.0f;
float target_y= chariot.DR16.Get_Right_Y()*4.0f;
float target_omega= chariot.DR16.Get_Left_X()*8.0f;
chassis.Set_Target_Velocity_X(target_x);
chassis.Set_Target_Velocity_Y(target_y);
chassis.Set_Target_Omega(target_omega);
	}


  }
 