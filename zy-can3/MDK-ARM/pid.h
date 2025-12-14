#ifndef PID_H
#define PID_H

#include <stdint.h>

typedef struct {
    float kp;
    float ki;
    float kd;
    float set;    
    float fdb;    
    float out;   
    float integral;  
    float last_fdb;  
    float out_max;  
    float out_min;  
} PID_TypeDef;

void PID_Init(PID_TypeDef *pid, float kp, float ki, float kd, float out_max, float out_min);
float PID_Calc(PID_TypeDef *pid);

#endif
