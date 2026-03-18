#include "pid_controller.h"

/**
  * @brief 初始化PID控制器
  * @param pid PID控制器结构体指针
  * @param kp 比例系数
  * @param ki 积分系数
  * @param kd 微分系数
  * @param output_min 输出最小值
  * @param output_max 输出最大值
  */
void PID_Init(PID_Controller* pid, float kp, float ki, float kd, float output_min, float output_max)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->setpoint = 0.0f;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->output_min = output_min;
    pid->output_max = output_max;
}

/**
  * @brief PID计算
  * @param pid PID控制器结构体指针
  * @param feedback 反馈值
  * @return PID输出值
  */
float PID_Compute(PID_Controller* pid, float feedback)
{
    // 计算误差
    float error = pid->setpoint - feedback;
    
    // 积分项累加
    pid->integral += error;
    
    // 积分限幅，防止积分饱和
    if (pid->integral > 1000.0f) pid->integral = 1000.0f;
    if (pid->integral < -1000.0f) pid->integral = -1000.0f;
    
    // 微分项
    float derivative = error - pid->prev_error;
    
    // PID输出计算
    float output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
    
    // 输出限幅
    if (output > pid->output_max) output = pid->output_max;
    if (output < pid->output_min) output = pid->output_min;
    
    // 保存当前误差用于下次微分计算
    pid->prev_error = error;
    
    return output;
}

/**
  * @brief 重置PID控制器
  * @param pid PID控制器结构体指针
  */
void PID_Reset(PID_Controller* pid)
{
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
}

/**
  * @brief 设置PID控制器设定点
  * @param pid PID控制器结构体指针
  * @param setpoint 设定点
  */
void PID_SetSetpoint(PID_Controller* pid, float setpoint)
{
    pid->setpoint = setpoint;
}