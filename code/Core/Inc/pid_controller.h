#ifndef __PID_CONTROLLER_H
#define __PID_CONTROLLER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
 * @brief PID控制器结构体
 */
typedef struct {
    float kp;           // 比例系数
    float ki;           // 积分系数
    float kd;           // 微分系数
    float setpoint;     // 设定点
    float integral;     // 积分项
    float prev_error;   // 上一次误差
    float output_min;   // 输出最小值
    float output_max;   // 输出最大值
} PID_Controller;

/**
 * @brief 初始化PID控制器
 * @param pid PID控制器结构体指针
 * @param kp 比例系数
 * @param ki 积分系数
 * @param kd 微分系数
 * @param output_min 输出最小值
 * @param output_max 输出最大值
 */
void PID_Init(PID_Controller* pid, float kp, float ki, float kd, float output_min, float output_max);

/**
 * @brief PID计算
 * @param pid PID控制器结构体指针
 * @param feedback 反馈值
 * @return PID输出值
 */
float PID_Compute(PID_Controller* pid, float feedback);

/**
 * @brief 重置PID控制器
 * @param pid PID控制器结构体指针
 */
void PID_Reset(PID_Controller* pid);

/**
 * @brief 设置PID控制器设定点
 * @param pid PID控制器结构体指针
 * @param setpoint 设定点
 */
void PID_SetSetpoint(PID_Controller* pid, float setpoint);

#ifdef __cplusplus
}
#endif

#endif /* __PID_CONTROLLER_H */