#ifndef __PID_H
#define __PID_H

#include "main.h"
#include "can.h"

/* PID结构体 */
typedef struct {
    float target_val;   
    float actual_val;       
    float err;         
    float err_last;     
    float err_sum;      
    float Kp, Ki, Kd;   
    float output;        
    float output_max;    
    float integral_max;  
} PID_TypeDef;

typedef struct {
    uint16_t ecd;         
    uint16_t last_ecd;    
    int16_t speed_rpm;   
    int16_t real_current;  
    int16_t given_current; 
    uint8_t temperature;   
    int32_t total_angle;  
    int32_t round_cnt;     // 圈数计数
    float angle_deg;       // 角度（度）-720° to 720°
} Motor_Feedback_TypeDef;

void PID_Init(void);
float PID_Calculate(PID_TypeDef *pid, float target, float actual);
void Motor_Feedback_Update(Motor_Feedback_TypeDef *mf, uint8_t *rx_data);
void CAN_Send_Current(int16_t current1, int16_t current2, int16_t current3, int16_t current4);
void VOFA_ParseCommand(char* cmd);
void VOFA_SendData(void);
void Angle_Normalization(float *angle);


extern PID_TypeDef angle_pid;    // 角度环PID
extern PID_TypeDef speed_pid;    // 速度环PID
extern Motor_Feedback_TypeDef motor_fb;
extern int16_t current_to_send; 

#endif
