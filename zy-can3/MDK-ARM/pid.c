#include "pid.h"


void PID_Init(PID_TypeDef *pid, float kp, float ki, float kd, float out_max, float out_min) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->set = 0.0f;
    pid->fdb = 0.0f;
    pid->out = 0.0f;
    pid->integral = 0.0f;
    pid->last_fdb = 0.0f;
    pid->out_max = out_max;
    pid->out_min = out_min;
}

float PID_Calc(PID_TypeDef *pid) {
    float err = pid->set - pid->fdb;
    
    // 积分计算
    pid->integral += err;
    // 积分限幅
    if (pid->integral > pid->out_max) pid->integral = pid->out_max;
    if (pid->integral < pid->out_min) pid->integral = pid->out_min;
    
    // PID输出计算
    pid->out = pid->kp*err + pid->ki * pid->integral + pid->kd * (pid->fdb - pid->last_fdb);
    
    // 输出限幅
    if (pid->out > pid->out_max) pid->out=pid->out_max;
    if (pid->out < pid->out_min) pid->out = pid->out_min;
    
    // 更新上一次反馈值
    pid->last_fdb = pid->fdb;
    
    return pid->out;
}
