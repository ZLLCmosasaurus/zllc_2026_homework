#include "task.h"
#include "drv_can.h"
#include "drv_tim.h"
#include "dvc_djimotor.h"
#include "dvc_dr16.h"
#include "ita_chariot.h"

void forward(float rad);
int count=0;
Class_Chariot chariot;


void Class_Chariot::Init(float __Dead_Zone)
{
    #ifdef CHASSIS
    
        //裁判系统
        Referee.Init(&huart6);

        //底盘
        Chassis.Referee = &Referee;
        Chassis.Init();

        //底盘随动PID环初始化
        PID_Chassis_Fllow.Init(6.0f, 0.0f, 0.1f, 0.0f, 10.0f, 10.0f,0.0f,0.0f,0.0f,0.001f,0.01f);

        //yaw电机canid初始化  只获取其编码器值用于底盘随动，并不参与控制
        Motor_Yaw.Init(&hcan2, DJI_Motor_ID_0x205);

    #elif defined(GIMBAL)
        
        Chassis.Set_Velocity_X_Max(4.0f);
        Chassis.Set_Velocity_Y_Max(4.0f);

        //遥控器离线控制 状态机
        #ifdef USE_DR16
        FSM_Alive_Control.Chariot = this;
        FSM_Alive_Control.Init(5, 0);

        DR16.Init(&huart3,&huart1);
        #endif

        #ifdef USE_VT13
        FSM_Alive_Control_VT13.Chariot = this;
        FSM_Alive_Control_VT13.Init(5,0);
        #endif

        //遥控器死区
        Dead_Zone = __Dead_Zone;   

//        //云台
//        Gimbal.Init();
//        Gimbal.MiniPC = &MiniPC;

//        //发射机构
//        Booster.Referee = &Referee;
//        Booster.Init();
//				
//        //上位机
//        MiniPC.Init(&MiniPC_USB_Manage_Object);
//        MiniPC.IMU = &Gimbal.Boardc_BMI;
//        MiniPC.Referee = &Referee;

    #endif
}





void Chassis_Device_CAN1_Callback(Struct_CAN_Rx_Buffer *CAN_RxMessage)
{
    switch (CAN_RxMessage->Header.StdId)
    {
         case (0x201):
        {
            chariot.Chassis.Motor_Wheel[0].CAN_RxCpltCallback(CAN_RxMessage->Data);
        }
        break;
        case (0x202):
        {
            chariot.Chassis.Motor_Wheel[1].CAN_RxCpltCallback(CAN_RxMessage->Data);
        }
        break;
        case (0x203):
        {
            chariot.Chassis.Motor_Wheel[2].CAN_RxCpltCallback(CAN_RxMessage->Data);
        }
        break;
        case (0x204):
        {
            chariot.Chassis.Motor_Wheel[3].CAN_RxCpltCallback(CAN_RxMessage->Data);
        }
    }
}







void Task1ms_TIM5_Callback() //every 1ms conduct
{
  //motor1
  chariot.Chassis.Motor_Wheel[0].TIM_PID_PeriodElapsedCallback();
  chariot.Chassis.Motor_Wheel[1].TIM_PID_PeriodElapsedCallback();
  //motor3
  chariot.Chassis.Motor_Wheel[2].TIM_PID_PeriodElapsedCallback();
  //motor4
  chariot.Chassis.Motor_Wheel[3].TIM_PID_PeriodElapsedCallback();
  //can 
  TIM_CAN_PeriodElapsedCallback();
	//CAN_Send_Data(&hcan1,0x200,CAN1_0x200_Tx_Data,8);
  //dr16
  count++;
  if(count==15) //15ms
  {
    chariot.DR16.TIM1msMod50_Alive_PeriodElapsedCallback();
    count=0;
  }
  chariot.FSM_Alive_Control.Reload_TIM_Status_PeriodElapsedCallback();
//  if(Motor_Yaw.Get_DJI_Motor_Status() == DJI_Motor_Status_ENABLE)


}




//uart disable
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
            if(Chariot->DR16.Get_DR16_Status() == DR16_Status_ENABLE)
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
            Chariot->Booster.Set_Booster_Control_Type(Booster_Control_Type_DISABLE);
            Chariot->Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_DISABLE);
            Chariot->Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE);

            if(Chariot->DR16.Get_DR16_Status() == DR16_Status_ENABLE)
            {
                Chariot->Chassis.Set_Chassis_Control_Type(Chariot->Get_Pre_Chassis_Control_Type());
                Chariot->Gimbal.Set_Gimbal_Control_Type(Chariot->Get_Pre_Gimbal_Control_Type());
                Status[Now_Status_Serial].Time = 0;
                Set_Status(2);
            }

            // 遥控器中途断联导致错误离线 跳转到 遥控器串口错误状态
            if (huart3.ErrorCode)
            {
                Status[Now_Status_Serial].Time = 0;
                Set_Status(4);
            }
    chariot.Chassis.Motor_Wheel[0].Set_Target_Omega_Radian(0);
    chariot.Chassis.Motor_Wheel[1].Set_Target_Omega_Radian(0);
    chariot.Chassis.Motor_Wheel[2].Set_Target_Omega_Radian(0);
    chariot.Chassis.Motor_Wheel[3].Set_Target_Omega_Radian(0);

         
            
        }
        break;
        // 遥控器在线状态
        case (2):
        {
            //转移为 刚离线状态
            if(Chariot->DR16.Get_DR16_Status() == DR16_Status_DISABLE)
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
            Chariot->Set_Pre_Chassis_Control_Type(Chariot->Chassis.Get_Chassis_Control_Type());
            Chariot->Set_Pre_Gimbal_Control_Type(Chariot->Gimbal.Get_Gimbal_Control_Type());

            //无条件转移到 离线检测状态
            Status[Now_Status_Serial].Time = 0;
            Set_Status(0);
					
			chariot.Chassis.Motor_Wheel[0].Set_Target_Omega_Radian(0);
			chariot.Chassis.Motor_Wheel[1].Set_Target_Omega_Radian(0);
			chariot.Chassis.Motor_Wheel[2].Set_Target_Omega_Radian(0);
			chariot.Chassis.Motor_Wheel[3].Set_Target_Omega_Radian(0);
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










void DR16_UART3_Callback(uint8_t *Buffer, uint16_t Length)
{

    chariot.DR16.DR16_UART_RxCpltCallback(Buffer);

}

  void Task_Init()
  { 

    
    //can
    CAN_Init(&hcan1, Chassis_Device_CAN1_Callback);
    //tim5
    TIM_Init(&htim5, Task1ms_TIM5_Callback);
    HAL_TIM_Base_Start_IT(&htim5);
    //chariot 
    
    chariot.Chassis.Motor_Wheel[0].Init(&hcan1,DJI_Motor_ID_0x201,DJI_Motor_Control_Method_OMEGA,19.0f);
    chariot.Chassis.Motor_Wheel[0].PID_Omega.Init(200.0f,0.01f,0.0f,0.0f,10000.0f,16000.0f);//speed pid init
    chariot.Chassis.Motor_Wheel[1].Init(&hcan1,DJI_Motor_ID_0x202,DJI_Motor_Control_Method_OMEGA,19.0f);
    chariot.Chassis.Motor_Wheel[1].PID_Omega.Init(200.0f,0.01f,0.0f,0.0f,10000.0f,16000.0f);//speed pid init
		chariot.Chassis.Motor_Wheel[2].Init(&hcan1,DJI_Motor_ID_0x203,DJI_Motor_Control_Method_OMEGA,19.0f);
    chariot.Chassis.Motor_Wheel[2].PID_Omega.Init(200.0f,0.01f,0.0f,0.0f,10000.0f,16000.0f);//speed pid init
    chariot.Chassis.Motor_Wheel[3].Init(&hcan1,DJI_Motor_ID_0x204,DJI_Motor_Control_Method_OMEGA,19.0f);
    chariot.Chassis.Motor_Wheel[3].PID_Omega.Init(200.0f,0.01f,0.0f,0.0f,10000.0f,16000.0f);//speed pid init
    //dr_16
   // chariot.DR16.Init(&huart3,&huart1);
    UART_Init(&huart3, DR16_UART3_Callback, 18);

    chariot.Init();
    
    
  }


void forward(float rad)
{
  chariot.Chassis.Motor_Wheel[0].Set_Target_Omega_Radian(rad);
  chariot.Chassis.Motor_Wheel[1].Set_Target_Omega_Radian(-rad);
  chariot.Chassis.Motor_Wheel[2].Set_Target_Omega_Radian(-rad);
  chariot.Chassis.Motor_Wheel[3].Set_Target_Omega_Radian(rad);
}

void backward(float rad)
{
	chariot.Chassis.Motor_Wheel[0].Set_Target_Omega_Radian(-rad);
  chariot.Chassis.Motor_Wheel[1].Set_Target_Omega_Radian(rad);
  chariot.Chassis.Motor_Wheel[2].Set_Target_Omega_Radian(rad);
  chariot.Chassis.Motor_Wheel[3].Set_Target_Omega_Radian(-rad);
}

void turn_left(float rad)
{
  chariot.Chassis.Motor_Wheel[0].Set_Target_Omega_Radian(-rad);
  chariot.Chassis.Motor_Wheel[1].Set_Target_Omega_Radian(-rad);
  chariot.Chassis.Motor_Wheel[2].Set_Target_Omega_Radian(-rad);
  chariot.Chassis.Motor_Wheel[3].Set_Target_Omega_Radian(-rad);

}

void turn_right(float rad)
{
  chariot.Chassis.Motor_Wheel[0].Set_Target_Omega_Radian(rad);
  chariot.Chassis.Motor_Wheel[1].Set_Target_Omega_Radian(rad);
  chariot.Chassis.Motor_Wheel[2].Set_Target_Omega_Radian(rad);
  chariot.Chassis.Motor_Wheel[3].Set_Target_Omega_Radian(rad);

}

void trans_left(float rad)
{
  chariot.Chassis.Motor_Wheel[0].Set_Target_Omega_Radian(rad);
  chariot.Chassis.Motor_Wheel[1].Set_Target_Omega_Radian(rad);
  chariot.Chassis.Motor_Wheel[2].Set_Target_Omega_Radian(-rad);
  chariot.Chassis.Motor_Wheel[3].Set_Target_Omega_Radian(-rad);

}

void trans_right(float rad)
{
  chariot.Chassis.Motor_Wheel[0].Set_Target_Omega_Radian(-rad);
  chariot.Chassis.Motor_Wheel[1].Set_Target_Omega_Radian(-rad);
  chariot.Chassis.Motor_Wheel[2].Set_Target_Omega_Radian(rad);
  chariot.Chassis.Motor_Wheel[3].Set_Target_Omega_Radian(rad);

}


  void Task_Loop()
  {
  if(chariot.DR16.Get_DR16_Status()==DR16_Status_ENABLE)
  {
      if(chariot.DR16.Get_Left_Y()>0.5)
    forward(50);
   else if(chariot.DR16.Get_Left_Y()<-0.5) 
    backward(50);
   else if(chariot.DR16.Get_Left_X()<-0.5) 
    turn_left(50);
  else if(chariot.DR16.Get_Left_X()>0.5) 
    turn_right(50);
  else if(chariot.DR16.Get_Right_X()<-0.5) 
    trans_left(50);
  else if(chariot.DR16.Get_Right_X()>0.5) 
    trans_right(50);
	else
    forward(0);
		         
  }
 
 }


 


  
  
  
    

  