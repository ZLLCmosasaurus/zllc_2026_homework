#ifndef __PID_H
#define __PID_H

typedef struct {
    float target_speed; // 目标速度
    float Kp;           // 比例系数
    float Ki;           // 积分系数
    float Kd;           // 微分系数
    float error_k;      // 当前误差 e(k)
    float error_k_1;    // 上一次误差 e(k-1)
    float error_k_2;    // 上上次误差 e(k-2)
    float pwm_max;      // PWM最大值
    float pwm_min;      // PWM最小值
    float last_pwm_output;  // 历史PID输出
} PID_PWM_TypeDef;

void PID_Init(PID_PWM_TypeDef *pid, float kp, float ki, float kd, float max_out, float min_out);
float PID_Calculate(PID_PWM_TypeDef *pid, float setpoint, float actual_speed);

#endif // !__PID_H
