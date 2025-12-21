#include "pid.h"

void PID_Init(PID_Controller_t *pid, float kp, float ki, float kd, 
              float output_max, float output_min)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->target = 0.0f;
    pid->current = 0.0f;
    pid->error = 0.0f;
    pid->last_error = 0.0f;
    pid->integral = 0.0f;
    pid->output = 0.0f;
    pid->output_max = output_max;
    pid->output_min = output_min;
    pid->integral_max = output_max * 0.5f;  // 积分限幅为输出限幅的一半
    pid->integral_min = output_min * 0.5f;
}

void PID_SetTarget(PID_Controller_t *pid, float target)
{
    pid->target = target;
}

float PID_Calculate(PID_Controller_t *pid, float current)
{
    float p_out, i_out, d_out;
    
    pid->current = current;
    pid->error = pid->target - pid->current;
    
    p_out = pid->kp * pid->error;
    
    pid->integral += pid->error;
    
    if (pid->integral > pid->integral_max) 
    {
        pid->integral = pid->integral_max;
    }
    else if (pid->integral < pid->integral_min) 
    {
        pid->integral = pid->integral_min;
    }
    
    i_out = pid->ki * pid->integral;
    
    d_out = pid->kd * (pid->error - pid->last_error);
    
    pid->output = p_out + i_out + d_out;
    
    if (pid->output > pid->output_max) 
    {
        pid->output = pid->output_max;
    } 
    else if (pid->output < pid->output_min) 
    {    
      pid->output = pid->output_min;
    }
    
    pid->last_error = pid->error;
    
    return pid->output;
}

void PID_Reset(PID_Controller_t *pid)
{
    pid->error = 0.0f;
    pid->last_error = 0.0f;
    pid->integral = 0.0f;
    pid->output = 0.0f;
}

/* USER CODE END 1 */

