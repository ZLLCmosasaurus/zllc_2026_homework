/* Includes ------------------------------------------------------------------*/
#include "tsk_config_and_callback.h"
#include "drv_tim.h"
#include "config.h"
#include "drv_uart.h"
#include "math.h"
#include "drv_dwt.h"
#include "dvc_referee.h"

/* Private variables ---------------------------------------------------------*/

uint32_t init_finished = 0;
bool start_flag = 0;
static uint8_t referee_alive_counter = 0;

Class_Referee referee;

void Task1ms_TIM5_Callback()
{
    init_finished++;
    if(init_finished>2000)
    start_flag=1;

    if(start_flag==1)
    {
        referee_alive_counter++;
        if (referee_alive_counter >= 100) // 每 100ms 检查一次保活
        {
            referee_alive_counter = 0;
            referee.TIM1msMod50_Alive_PeriodElapsedCallback();
        }
    }
}

void Referee_Data_Callback(uint8_t *Rx_Buffer, uint16_t Rx_Length)
{
    referee.UART_RxCpltCallback(Rx_Buffer, Rx_Length);
}

/**
 * @brief 初始化任务
 *
 */
extern "C" void Task_Init()
{  
    DWT_Init(168);
    
    referee.Init(&huart6, 0xA5);

    UART_Init(&huart6, Referee_Data_Callback, 128);
    
    TIM_Init(&htim5, Task1ms_TIM5_Callback);

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