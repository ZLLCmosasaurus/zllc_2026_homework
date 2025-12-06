#include "motor_simple.h"
#include "SEGGER_RTT.h"
#include <math.h>

extern CAN_HandleTypeDef hcan;

// 全局变量定义
int16_t target_rpm = 500;     // 默认目标转速
float target_degree = 0.0f;   // 默认目标角度
int16_t actual_rpm = 0;
float actual_degree = 0.0f;
int16_t output_current = 0;
uint8_t motor_online = 0;
uint32_t can_tx_count = 0;
uint32_t can_rx_count = 0;
uint32_t last_motor_time = 0;

ControlMode control_mode = MODE_POSITION;  // 默认速度模式

Motor_Data motor = {0};

// 速度环PID参数
PID_Param vel_pid = {3.0f, 0.1f, 0.0f, 0, 0};

// 位置环PID参数（新增）
PID_Param pos_pid = {6.0f, 0.122f, 0.9f, 0, 0};

// 内部变量
static int16_t last_encoder = 0;
static int32_t encoder_turns = 0;  // 圈数计数

// 常量
#define ENCODER_MAX 8191.0f
#define DEGREE_PER_TICK (360.0f / ENCODER_MAX)

void motor_init(void)
{
    target_rpm = 500;
    target_degree = 0.0f;
    actual_rpm = 0;
    actual_degree = 0.0f;
    output_current = 0;
    motor_online = 0;
    control_mode = MODE_POSITION;
    
    vel_pid.integral = 0;
    vel_pid.last_error = 0;
    
    pos_pid.integral = 0;
    pos_pid.last_error = 0;
    
    last_encoder = 0;
    encoder_turns = 0;
    motor.total_encoder = 0;
    
    SEGGER_RTT_WriteString(0, "Motor Control Started\n");
    SEGGER_RTT_printf(0, "Speed PID: Kp=%.2f, Ki=%.2f, Kd=%.2f\n", 
                     vel_pid.kp, vel_pid.ki, vel_pid.kd);
    SEGGER_RTT_printf(0, "Pos PID: Kp=%.2f, Ki=%.2f, Kd=%.2f\n", 
                     pos_pid.kp, pos_pid.ki, pos_pid.kd);
    SEGGER_RTT_WriteString(0, "Control Mode: SPEED (default)\n");
}

// 设置控制模式
void set_control_mode(ControlMode mode)
{
    control_mode = mode;
    
    if (mode == MODE_POSITION) {
        SEGGER_RTT_WriteString(0, "Mode: POSITION\n");
    } else {
        SEGGER_RTT_WriteString(0, "Mode: SPEED\n");
    }
}

// 角度计数清零
void reset_angle_count(void)
{
    encoder_turns = 0;
    motor.total_encoder = 0;
    actual_degree = 0.0f;
    motor.angle_degree = 0.0f;
    
    SEGGER_RTT_WriteString(0, "Angle Count Reset\n");
}

// 调整位置环PID
void angle_pid_adjust(float kp, float ki, float kd)
{
    pos_pid.kp = kp;
    pos_pid.ki = ki;
    pos_pid.kd = kd;
    
    SEGGER_RTT_printf(0, "Pos PID Updated: %.2f, %.2f, %.2f\n", kp, ki, kd);
}

void send_current_to_motor(int16_t current)
{
    uint8_t data[8] = {0};
    CAN_TxHeaderTypeDef tx_header;
    uint32_t mailbox;
    
    // 限制电流范围
    if (current > 10000) current = 10000;
    if (current < -10000) current = -10000;
    
    tx_header.StdId = 0x200;
    tx_header.ExtId = 0;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8;
    tx_header.TransmitGlobalTime = DISABLE;
    
    data[0] = (current >> 8) & 0xFF;
    data[1] = current & 0xFF;
    
    if (HAL_CAN_AddTxMessage(&hcan, &tx_header, data, &mailbox) == HAL_OK) {
        can_tx_count++;
        output_current = current;
    }
}

void motor_process_data(uint16_t can_id, uint8_t* data)
{
    if (can_id >= 0x201 && can_id <= 0x208) {
        // 解析数据
        int16_t encoder = (data[0] << 8) | data[1];
        int16_t speed = (data[2] << 8) | data[3];
        int16_t current = (data[4] << 8) | data[5];
        
        // 处理多圈角度计算
        int16_t delta;
        
        // 计算编码器变化量，处理溢出
        if (last_encoder > 6000 && encoder < 2000) {
            // 正转溢出
            delta = (8192 - last_encoder) + encoder;
            encoder_turns++;
        } else if (last_encoder < 2000 && encoder > 6000) {
            // 反转溢出
            delta = -((8192 - encoder) + last_encoder);
            encoder_turns--;
        } else {
            delta = encoder - last_encoder;
        }
        
        // 更新总编码器值
        motor.total_encoder += delta;
        
        // 计算实际角度（度）
        actual_degree = encoder_turns * 360.0f + encoder * DEGREE_PER_TICK;
        
        // 保存数据
        motor.angle = encoder;
        motor.speed_rpm = speed;
        motor.actual_current = current;
        motor.angle_degree = actual_degree;
        motor.online = 1;
        
        // 更新全局变量
        actual_rpm = speed;
        actual_degree = motor.angle_degree;
        motor_online = 1;
        last_motor_time = HAL_GetTick();
        last_encoder = encoder;
        
        // 调试输出
        static uint32_t last_debug = 0;
        if (HAL_GetTick() - last_debug > 300) {
            SEGGER_RTT_printf(0, "MOTOR: %.1f°, %dRPM, %dA\n", 
                             actual_degree, speed, current);
            last_debug = HAL_GetTick();
        }
    }
}

// 计算位置环PID（返回目标转速）
static int16_t calculate_position_pid(void)
{
    float error = target_degree - actual_degree;
    
    // P项
    float p_term = pos_pid.kp * error;
    
    // I项
    pos_pid.integral += error;
    if (pos_pid.integral > 1000) pos_pid.integral = 1000;
    if (pos_pid.integral < -1000) pos_pid.integral = -1000;
    float i_term = pos_pid.ki * pos_pid.integral;
    
    // D项
    float d_term = pos_pid.kd * (error - pos_pid.last_error);
    pos_pid.last_error = error;
    
    // 计算目标转速
    float target_speed = p_term + i_term + d_term;
    
    // 限幅（±3000 RPM）
    if (target_speed > 3000.0f) target_speed = 3000.0f;
    if (target_speed < -3000.0f) target_speed = -3000.0f;
    
    return (int16_t)target_speed;
}

// 计算速度环PID（返回电流）
static int16_t calculate_velocity_pid(int16_t target_speed)
{
    int16_t error = target_speed - actual_rpm;
    
    float p_term = vel_pid.kp * error;
    
    vel_pid.integral += error;
    if (vel_pid.integral > 1000) vel_pid.integral = 1000;
    if (vel_pid.integral < -1000) vel_pid.integral = -1000;
    float i_term = vel_pid.ki * vel_pid.integral;
    
    float d_term = vel_pid.kd * (error - vel_pid.last_error);
    vel_pid.last_error = error;
    
    float output = p_term + i_term + d_term;
    
    if (output > 10000) output = 10000;
    if (output < -10000) output = -10000;
    
    return (int16_t)output;
}

void update_pid_control(void)
{
    if (!motor_online) {
        static uint32_t last_zero_time = 0;
        if (HAL_GetTick() - last_zero_time > 100) {
            send_current_to_motor(0);
            last_zero_time = HAL_GetTick();
        }
        return;
    }
    
    int16_t target_speed = 0;
    int16_t current_output = 0;
    
    if (control_mode == MODE_POSITION) {
        // 位置环：外环计算目标速度，内环计算电流
        target_speed = calculate_position_pid();
        current_output = calculate_velocity_pid(target_speed);
        
        // 调试输出
        static uint32_t last_pos_debug = 0;
        if (HAL_GetTick() - last_pos_debug > 400) {
            SEGGER_RTT_printf(0, "POS: T=%.1f°, A=%.1f°, TS=%d, Out=%d\n",
                             target_degree, actual_degree, target_speed, current_output);
            last_pos_debug = HAL_GetTick();
        }
    } else {
        // 速度环
        current_output = calculate_velocity_pid(target_rpm);
        
        static uint32_t last_vel_debug = 0;
        if (HAL_GetTick() - last_vel_debug > 400) {
            SEGGER_RTT_printf(0, "VEL: T=%d, A=%d, Out=%d\n",
                             target_rpm, actual_rpm, current_output);
            last_vel_debug = HAL_GetTick();
        }
    }
    
    send_current_to_motor(current_output);
}