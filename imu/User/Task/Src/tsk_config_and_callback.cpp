#include "tsk_config_and_callback.h"
#include "drv_tim.h"
#include "dvc_boardc_bmi088.h"
#include "ita_chariot.h"
#include "dvc_imu.h"
#include "config.h"

uint32_t init_finished =0 ;
bool start_flag=0;
Class_IMU imu;

float test_imu_roll = 0;
float test_imu_pitch = 0;
float test_imu_yaw = 0;

/**
 * @brief SPI1回调函数
 *
 * @param Tx_Buffer SPI1发送的消息
 * @param Rx_Buffer SPI1接收的消息
 * @param Length 长度
 */
void Device_SPI2_Callback(uint8_t *Tx_Buffer, uint8_t *Rx_Buffer, uint16_t Length)
{

}

/**
 * @brief TIM5任务回调函数
 *
 */
extern Referee_Rx_A_t CAN3_Chassis_Rx_Data_A;
void Task1ms_TIM5_Callback()
{
    init_finished++;
    if(init_finished>2000)
    start_flag=1;

    if(start_flag==1)
    {

        imu.TIM_Calculate_PeriodElapsedCallback();

        test_imu_roll = imu.Get_Angle_Roll();
        test_imu_pitch = imu.Get_Angle_Pitch();
        test_imu_yaw = imu.Get_Angle_Yaw();
    }
}

/**
 * @brief 初始化任务
 *
 */
extern "C" void Task_Init()
{  

    DWT_Init(480);

     //c板陀螺仪spi外设
    SPI_Init(&hspi2,Device_SPI2_Callback);

    imu.Init();

    //定时器循环任务
    TIM_Init(&htim5, Task1ms_TIM5_Callback);

    HAL_TIM_Base_Start_IT(&htim5);
}
