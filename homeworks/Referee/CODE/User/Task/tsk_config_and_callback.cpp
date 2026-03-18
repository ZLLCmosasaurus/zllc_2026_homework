/**
 * @brief TIM开头的默认任务均1ms, 特殊任务需额外标记时间
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "tsk_config_and_callback.h"
#include "drv_tim.h"
#include "ita_chariot.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

uint32_t init_finished = 0;
bool start_flag = false;
// 机器人控制对象
Class_Referee Referee;

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief UART裁判系统回调函数
 *
 * @param Buffer UART收到的消息
 * @param Length 长度
 */
void Referee_UART6_Callback(uint8_t *Buffer, uint16_t Length)
{
    Referee.UART_RxCpltCallback(Buffer, Length);
}

/**
 * @brief TIM4任务回调函数
 *
 */
void Task100us_TIM4_Callback()
{
}

/**
 * @brief TIM5任务回调函数
 *
 */

void Task1ms_TIM5_Callback()
{
    init_finished++;
    if (init_finished > 2000)
        start_flag = true;

    /************ 判断设备在线状态判断 50ms (所有device:电机，遥控器，裁判系统等) ***************/

    static uint8_t mod50 = 0;
    if (++mod50 >= 50)
    {
        Referee.TIM1msMod50_Alive_PeriodElapsedCallback();
        mod50 = 0;
    }

    /****************************** 交互层回调函数 1ms *****************************************/
    if (start_flag)
    {
        /****************************** 驱动层回调函数 1ms *****************************************/
        // 统一打包发送
        TIM_UART_PeriodElapsedCallback();
        Referee.UART_Tx_Referee_UI();
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
    // 裁判系统
    UART_Init(&huart6, Referee_UART6_Callback, 128); // 并未使用环形队列 尽量给长范围增加检索时间 减少丢包

    // 定时器循环任务
    TIM_Init(&htim4, Task100us_TIM4_Callback);
    TIM_Init(&htim5, Task1ms_TIM5_Callback);

    /********************************* 设备层初始化 *********************************/

    // 设备层集成在交互层初始化中，没有显视地初始化

    /********************************* 交互层初始化 *********************************/

    Referee.Init(&huart6);

    /********************************* 使能调度时钟 *********************************/

    HAL_TIM_Base_Start_IT(&htim4);
    HAL_TIM_Base_Start_IT(&htim5);
}

/**
 * @brief 前台循环任务
 *
 */
extern "C" void Task_Loop()
{
}
