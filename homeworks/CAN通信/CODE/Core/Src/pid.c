#include "pid.h"

void PID_Init(PID_TypeDef *pid, float Kp, float Ki, float Kd, float output_max, float output_min)
{
    pid->Kp         = Kp;
    pid->Ki         = Ki;
    pid->Kd         = Kd;
    pid->target     = 0;
    pid->actual     = 0;
    pid->err        = 0;
    pid->last_err   = 0;
    pid->total_err  = 0;
    pid->output     = 0;
    pid->output_max = output_max;
    pid->output_min = output_min;
}

float PID_Calculate(PID_TypeDef *pid, float actual)
{
    pid->actual = actual;
    pid->err    = pid->target - pid->actual;
    pid->total_err += pid->err;
    pid->output   = pid->Kp * pid->err + pid->Ki * pid->total_err + pid->Kd * (pid->err - pid->last_err);
    pid->last_err = pid->err;
    if (pid->output < pid->output_min) {
        pid->output = pid->output_min;
    }
    if (pid->output > pid->output_max) {
        pid->output = pid->output_max;
    }
    return pid->output;
}
