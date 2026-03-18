/* Includes ------------------------------------------------------------------*/
#include "tsk_config_and_callback.h"
#include "drv_tim.h"
#include "config.h"
#include "drv_uart.h"
#include "drv_can.h"
#include "dvc_dr16.h"
#include "dvc_djimotor.h"
#include "math.h"
#include "drv_dwt.h"

/* Private macros ------------------------------------------------------------*/
// 机器人参数 (根据实际车体调整)
#define ROBOT_MAX_RPM 800.0f  // 机器人最大期望转速 (RPM)

/* Private variables ---------------------------------------------------------*/

uint32_t init_finished = 0;
bool start_flag = 0;

Class_DR16 DR16;

// 0:右前, 1:左前, 2:左后, 3:右后
Class_DJI_Motor_C620 Motor_Chassis[4];

/* Private function declarations ---------------------------------------------*/

/**
 * @brief 遥控器串口接收回调桥接函数
 * UART驱动是C语言回调，需要桥接到C++对象的成员函数
 */
uint32_t Bridge_Call_Count = 0;

void DR16_Callback_Bridge(uint8_t *Buffer, uint16_t Length)
{
    Bridge_Call_Count++;

    DR16.DR16_UART_RxCpltCallback(Buffer);
}
/**
 * @brief Chassis_CAN1回调函数
 * 将CAN收到的数据分发给对应的电机对象
 */
#ifdef CHASSIS
void Chassis_Device_CAN1_Callback(Struct_CAN_Rx_Buffer *CAN_RxMessage)
{
    switch (CAN_RxMessage->Header.StdId)
    {
        case (0x201):
            Motor_Chassis[0].CAN_RxCpltCallback(CAN_RxMessage->Data);
            break;
        case (0x202):
            Motor_Chassis[1].CAN_RxCpltCallback(CAN_RxMessage->Data);
            break;
        case (0x203):
            Motor_Chassis[2].CAN_RxCpltCallback(CAN_RxMessage->Data);
            break;
        case (0x204):
            Motor_Chassis[3].CAN_RxCpltCallback(CAN_RxMessage->Data);
            break;
        default:
            break;
    }
}
#endif

/**
 * @brief TIM5任务回调函数 (1ms中断)
 * 核心控制逻辑：遥控器保活 -> 运动解算 -> PID计算 -> 发送电流
 */
void Task1ms_TIM5_Callback()
{
    init_finished++;
    if(init_finished > 2000) start_flag = 1;

    /************ 1. 设备保活检测 (Heartbeat) ***************/
    static uint8_t ms = 0;
    ms++;
    if(ms == 50)
    {
        ms = 0;

        //检测遥控器是否掉线
        DR16.TIM1msMod50_Alive_PeriodElapsedCallback();
        
        // 检测电机是否掉线
        for(int i = 0; i < 4; i++)
        {
            Motor_Chassis[i].TIM_Alive_PeriodElapsedCallback();
        }
    }

    /**************** 2. 控制逻辑 *************************/
    if(start_flag == 1)
    {
        float target_vx = 0.0f; // 前后速度 (m/s 或 比例)
        float target_vy = 0.0f; // 左右平移速度
        float target_omega = 0.0f; // 旋转速度

        // 【安全开关】: 必须满足 1.遥控器在线 2.右拨杆处于最上方(UP)
        if(DR16.Get_DR16_Status() == DR16_Status_ENABLE && 
           DR16.Get_Right_Switch() == DR16_Switch_Status_UP)
        {
            // 获取遥控器摇杆数值 (驱动层已归一化为 -1.0 到 1.0)
            // 左摇杆Y轴控制前进后退
            target_vx = DR16.Get_Left_Y(); 
            // 左摇杆X轴控制左右平移
            target_vy = DR16.Get_Left_X(); 
            // 右摇杆X轴控制旋转
            target_omega = DR16.Get_Right_X(); 

            if(fabs(target_vx) < 0.05f) target_vx = 0;
            if(fabs(target_vy) < 0.05f) target_vy = 0;
            if(fabs(target_omega) < 0.05f) target_omega = 0;
        }
        else
        {
            // 安全状态：强制停车
            target_vx = 0;
            target_vy = 0;
            target_omega = 0;
        }

        // 麦克纳姆轮逆运动学解算
        // 假设布局：
        // 1(右前)  2(左前)
        // 4(右后)  3(左后) 
        // 注意：根据实际电机ID和安装方向，这里的正负号可能需要调整
        
        float wheel_speed[4];
        
        // 1号电机：左前 (LF) -> ID 201 -> 数组[0]
        // 逻辑：前进(+) + 右移(+) + 右转(+)
        wheel_speed[0] = target_vx + target_vy + target_omega;

        // 2号电机：右前 (RF) -> ID 202 -> 数组[1]
        // 逻辑：前进(+) + 右移(-) + 右转(-)
        wheel_speed[1] = -(target_vx - target_vy - target_omega);

        // 3号电机：右后 (RR) -> ID 203 -> 数组[2]
        // 逻辑：前进(+) + 右移(+) + 右转(-)
        wheel_speed[2] = -(target_vx + target_vy - target_omega);

        // 4号电机：左后 (LR) -> ID 204 -> 数组[3]
        // 逻辑：前进(+) + 右移(-) + 右转(+)
        wheel_speed[3] = target_vx - target_vy + target_omega;

        for(int i = 0; i < 4; i++)
        {
            // 将比例转换为目标角速度 (rad/s)
            float target_radps = wheel_speed[i] * ROBOT_MAX_RPM * RPM_TO_RADPS;

            // 设置目标速度
            Motor_Chassis[i].Set_Target_Omega_Radian(target_radps);
            
            // 执行PID计算
            Motor_Chassis[i].TIM_PID_PeriodElapsedCallback();
        }

        /**************** 3. 驱动层发送 *************************/ 

        TIM_CAN_PeriodElapsedCallback();
    }
}

/**
 * @brief 初始化任务
 *
 */
extern "C" void Task_Init()
{  
    DWT_Init(168);

    /********************************** 驱动层初始化 **********************************/

    CAN_Init(&hcan1, Chassis_Device_CAN1_Callback);

    DR16.Init(&huart3, &huart3);
    
    for(int i = 0; i < 4; i++)
    {
        Motor_Chassis[i].Init(&hcan1, (Enum_DJI_Motor_ID)(DJI_Motor_ID_0x201 + i), DJI_Motor_Control_Method_OMEGA);
        
        // 参数顺序: KP, KI, KD, KF, 积分限幅, 输出限幅
        Motor_Chassis[i].PID_Omega.Init(20.0f, 0.1f, 0.1f, 0.05f, 2000.0f, 3000.0f);
    }
        
    // 注意：需要传入桥接函数，因为 UART_Init 需要 C 函数指针
    UART_Init(&huart3, DR16_Callback_Bridge, 18); 

    TIM_Init(&htim5, Task1ms_TIM5_Callback);

    /********************************* 设备层初始化 *********************************/
    



    /********************************* 使能调度时钟 *********************************/

    // 开启中断
    HAL_TIM_Base_Start_IT(&htim5);
}

/**
 * @brief 前台循环任务
 *
 */
extern "C" void Task_Loop()
{
    // 空循环，所有逻辑在中断中处理
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/