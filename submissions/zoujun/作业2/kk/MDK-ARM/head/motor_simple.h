#ifndef __MOTOR_SIMPLE_H
#define __MOTOR_SIMPLE_H

#include "main.h"
#include "can.h"

// 全局变量声明
extern int16_t target_rpm;
extern float target_degree;      // 新增：目标角度（度）
extern int16_t actual_rpm;
extern float actual_degree;      // 新增：实际角度（度）
extern int16_t output_current;
extern uint8_t motor_online;
extern uint32_t can_tx_count;
extern uint32_t can_rx_count;
extern uint32_t last_motor_time;

// 控制模式
typedef enum {
    MODE_SPEED = 0,
    MODE_POSITION = 1
} ControlMode;

extern ControlMode control_mode;  // 新增：控制模式

// 电机数据结构
typedef struct {
    int16_t angle;          // 原始编码器值 (0-8191)
    int16_t speed_rpm;
    int16_t actual_current;
    float angle_degree;     // 新增：角度（度）
    int32_t total_encoder;  // 新增：总编码器计数（多圈）
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

extern PID_Param vel_pid;    // 速度环PID
extern PID_Param pos_pid;    // 新增：位置环PID

// 函数声明
void motor_init(void);
void send_current_to_motor(int16_t current);
void update_pid_control(void);
void motor_test_sequence(void);
void motor_process_data(uint16_t can_id, uint8_t* data);

// 新增函数
void set_control_mode(ControlMode mode);
void reset_angle_count(void);    // 角度计数清零
void angle_pid_adjust(float kp, float ki, float kd);

#endif /* __MOTOR_SIMPLE_H */