#include "motor_simple.h"
#include "SEGGER_RTT.h"
extern CAN_HandleTypeDef hcan;


// 全局变量定义
int16_t target_rpm = 500;  // 默认目标转速
int16_t actual_rpm = 0;
int16_t output_current = 0;
uint8_t motor_online = 0;
uint32_t can_tx_count = 0;
uint32_t can_rx_count = 0;
uint32_t last_motor_time = 0;

Motor_Data motor = {0};
PID_Param pid = {3.0f, 0.1f, 0.0f, 0, 0};  // 调整PID参数

void motor_init(void)
{
    target_rpm = 500;  // 设置默认目标转速
    actual_rpm = 0;
    output_current = 0;
    motor_online = 0;
    pid.integral = 0;
    pid.last_error = 0;
    
    SEGGER_RTT_WriteString(0, "Motor: C620 + M3508 Initialized\n");
    SEGGER_RTT_printf(0, "PID: Kp=%.2f, Ki=%.3f, Kd=%.3f\n", pid.kp, pid.ki, pid.kd);
    SEGGER_RTT_printf(0, "Default target_rpm = %d\n", target_rpm);
}

void send_current_to_motor(int16_t current)
{
    uint8_t data[8] = {0};
    CAN_TxHeaderTypeDef tx_header;
    uint32_t mailbox;
    
    // 限制电流范围 (-16384 ~ 16384)
    if (current > 10000) current = 10000;
    if (current < -10000) current = -10000;
    
    tx_header.StdId = 0x200;  // 控制ID
    tx_header.ExtId = 0;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8;
    tx_header.TransmitGlobalTime = DISABLE;
    
    // 设置电流值 (大端序)
    data[0] = (current >> 8) & 0xFF;  // 电机1电流高字节
    data[1] = current & 0xFF;         // 电机1电流低字节
    // data[2]-data[7] 为其他电机，保持为0
    
    if (HAL_CAN_AddTxMessage(&hcan, &tx_header, data, &mailbox) == HAL_OK) {
        can_tx_count++;
        output_current = current;
        
        // 减少调试输出频率
        static uint32_t last_tx_debug = 0;
        if (HAL_GetTick() - last_tx_debug > 200) {
            SEGGER_RTT_printf(0, "CAN_TX: Current=%d\n", current);
            last_tx_debug = HAL_GetTick();
        }
    } else {
        static uint32_t last_tx_error = 0;
        if (HAL_GetTick() - last_tx_error > 1000) {
            SEGGER_RTT_WriteString(0, "CAN_TX: FAILED!\n");
            last_tx_error = HAL_GetTick();
        }
    }
}

void motor_process_data(uint16_t can_id, uint8_t* data)
{
    // 检查是否是电机反馈ID (0x201-0x208)
    if (can_id >= 0x201 && can_id <= 0x208) {
        // 解析数据 (注意：数据是大端序)
        motor.angle = (data[0] << 8) | data[1];
        motor.speed_rpm = (data[2] << 8) | data[3];
        motor.actual_current = (data[4] << 8) | data[5];
        motor.online = 1;
        
        // 更新全局变量
        actual_rpm = motor.speed_rpm;
        motor_online = 1;
        last_motor_time = HAL_GetTick();
        
        // 调试输出（降低频率）
        static uint32_t last_debug = 0;
        if (HAL_GetTick() - last_debug > 300) {
            SEGGER_RTT_printf(0, "MOTOR: ID=0x%03X, RPM=%d, Current=%d\n", 
                             can_id, motor.speed_rpm, motor.actual_current);
            last_debug = HAL_GetTick();
        }
    }
}

void update_pid_control(void)
{
    // 如果电机离线，定期发送零电流保持通信
    if (!motor_online) {
        static uint32_t last_zero_time = 0;
        if (HAL_GetTick() - last_zero_time > 100) {
            send_current_to_motor(0);
            last_zero_time = HAL_GetTick();
        }
        return;
    }
    
    // 计算误差
    int16_t error = target_rpm - actual_rpm;
    
    // PID计算
    float p_term = pid.kp * error;
    
    // 积分项（带积分限幅）
    pid.integral += error;
    if (pid.integral > 1000) pid.integral = 1000;
    if (pid.integral < -1000) pid.integral = -1000;
    
    float i_term = pid.ki * pid.integral;
    float d_term = pid.kd * (error - pid.last_error);
    pid.last_error = error;
    
    // 计算输出
    float output = p_term + i_term + d_term;
    
    // 输出限幅
    if (output > 10000) output = 10000;
    if (output < -10000) output = -10000;
    
    // 发送电流
    send_current_to_motor((int16_t)output);
    
    // PID调试输出
    static uint32_t last_pid_debug = 0;
    if (HAL_GetTick() - last_pid_debug > 400) {
        SEGGER_RTT_printf(0, "PID: T=%d, A=%d, E=%d, Out=%d\n",
                         target_rpm, actual_rpm, error, (int16_t)output);
        last_pid_debug = HAL_GetTick();
    }
}