#ifndef __MOTOR_SIMPLE_H
#define __MOTOR_SIMPLE_H

#include "main.h"
#include "can.h"

// 全局变量声明
extern int16_t target_rpm;
extern int16_t actual_rpm;
extern int16_t output_current;
extern uint8_t motor_online;
extern uint32_t can_tx_count;
extern uint32_t can_rx_count;
extern uint32_t last_motor_time;

// 电机数据结构
typedef struct {
    int16_t angle;
    int16_t speed_rpm;
    int16_t actual_current;
    uint8_t online;
} Motor_Data;

extern Motor_Data motor;

// PID参数结构
typedef struct {
    float kp;
    float ki;
    float kd;
    float integral;
    int16_t last_error;
} PID_Param;

extern PID_Param pid;

// 函数声明
void motor_init(void);
void send_current_to_motor(int16_t current);
void update_pid_control(void);
void motor_test_sequence(void);
void motor_process_data(uint16_t can_id, uint8_t* data);
// 在 motor_simple.h 的末尾添加
void uart_command_handler(void);
void process_command(char* command);
void motor_monitor(void);
void status_led_update(void);
#endif /* __MOTOR_SIMPLE_H */