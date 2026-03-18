#ifndef __MOTOR_CONTROL_H__
#define __MOTOR_CONTROL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "pid.h"

typedef struct {
    uint16_t rotor_angle;        // 转子机械角度 (0-8191对应0-360°)
    int16_t rotor_speed;         // 转子转速 (rpm)
    int16_t torque_current;      // 实际转矩电流
    int8_t temperature;          // 电机温度
    uint32_t last_update_time;   // 最后更新时间
} MotorFeedback_t;

typedef struct {
    uint8_t motor_id;                    // 电机ID
    PID_Controller_t angle_pid;          // 角度环PID控制器（外环）
    PID_Controller_t speed_pid;          // 速度环PID控制器（内环）
    MotorFeedback_t feedback;            // 反馈数据
    
    float target_output_angle;           // 目标输出轴累计角度（度，可以是多圈）
    float current_output_angle;          // 当前输出轴角度（度，0-360归一化）
    float target_rotor_speed;            // 目标转子速度（rpm，由角度环输出）
    float current_rotor_speed;           // 当前转子速度（rpm）
    
    float total_output_angle;            // 累计输出轴角度（用于多圈计算）
    float last_rotor_angle;              // 上一次转子角度（用于角度累计）
    int32_t rotation_count;              // 旋转圈数计数
    
    uint32_t control_enable;             // 控制使能标志
} MotorControl_t;

void MotorControl_Init(MotorControl_t *motor, uint8_t motor_id);
void MotorControl_UpdateFeedback(MotorControl_t *motor, uint16_t rotor_angle, 
                                  int16_t rotor_speed, int16_t torque_current, 
                                  int8_t temperature);
void MotorControl_SetTargetAngle(MotorControl_t *motor, float target_angle_deg);
int16_t MotorControl_Calculate(MotorControl_t *motor);
void MotorControl_Reset(MotorControl_t *motor);
void MotorControl_SetEnable(MotorControl_t *motor, uint32_t enable);
void MotorControl_RotateTwoRoundsForward(MotorControl_t *motor);
void MotorControl_RotateTwoRoundsReverse(MotorControl_t *motor);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_CONTROL_H__ */

