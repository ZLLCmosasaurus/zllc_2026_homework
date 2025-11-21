#include "pid.h"

/**
 * @brief 初始化PID控制器参数
 * @param pid PID控制器结构体指针
 * @param kp 比例系数
 * @param ki 积分系数
 * @param kd 微分系数
 * @param max_out PWM最大输出值
 * @param min_out PWM最小输出值
 */
void PID_Init(PID_PWM_TypeDef *pid, float kp, float ki, float kd, float max_out, float min_out)
{
    pid->Kp           = kp;
    pid->Ki           = ki;
    pid->Kd           = kd;
    pid->pwm_max      = max_out;
    pid->pwm_min      = min_out;
    pid->target_speed = 0;
    pid->error_k      = 0;
    pid->error_k_1    = 0;
    pid->error_k_2    = 0;
    pid->last_pwm_output  = 0;
}

/**
 * @brief 执行PID计算并返回新的PWM占空比
 * @param pid PID控制器结构体指针
 * @param setpoint 目标转速
 * @param actual_speed 电机实际反馈转速
 * @return 新的PWM占空比
 */
float PID_Calculate(PID_PWM_TypeDef *pid, float setpoint, float actual_speed)
{
    float delta_pwm; // PWM增量
    float new_pwm;   // 新的PWM输出

    // 1. 更新目标值
    pid->target_speed = setpoint;

    // 2. 计算当前误差 e(k)
    pid->error_k = pid->target_speed - actual_speed;

    // 3. 计算增量式PID公式：
    // delta_pwm = Kp * [e(k) - e(k-1)]
    //           + Ki * e(k)
    //           + Kd * [e(k) - 2e(k-1) + e(k-2)]

    delta_pwm = pid->Kp * (pid->error_k - pid->error_k_1) + pid->Ki * pid->error_k + pid->Kd * (pid->error_k - 2.0f * pid->error_k_1 + pid->error_k_2);

    // 4. 更新新的PWM输出
    new_pwm = pid->last_pwm_output + delta_pwm;

    // 5. 输出限幅 (最重要的一步)
    if (new_pwm > pid->pwm_max) {
        new_pwm = pid->pwm_max;
    } else if (new_pwm < pid->pwm_min) {
        new_pwm = pid->pwm_min;
    }

    // 6. 更新历史状态
    // 将 e(k-1) 移动到 e(k-2)
    pid->error_k_2 = pid->error_k_1;
    // 将 e(k) 移动到 e(k-1)
    pid->error_k_1 = pid->error_k;
    // 存储本次的PWM输出，作为下一次计算的基础
    pid->last_pwm_output = new_pwm;

    // 7. 返回计算结果
    return new_pwm;
}