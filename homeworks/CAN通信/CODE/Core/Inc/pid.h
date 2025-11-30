#ifndef __PID_H
#define __PID_H

typedef struct
{
    float Kp, Ki, Kd;
    float target, actual;
    float err, last_err, total_err;
    float output, output_max, output_min;
} PID_TypeDef;

void PID_Init(PID_TypeDef *pid, float Kp, float Ki, float Kd, float output_max, float output_min);

float PID_Calculate(PID_TypeDef *pid, float actual);

#endif // !__PID_H