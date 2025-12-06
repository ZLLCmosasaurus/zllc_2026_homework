#include "pid.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"


PID_TypeDef angle_pid = {0};
PID_TypeDef speed_pid = {0};
Motor_Feedback_TypeDef motor_fb = {0};
int16_t current_to_send = 0;

extern CAN_HandleTypeDef hcan;
extern UART_HandleTypeDef huart1;
void Angle_Normalization(float *angle) {
    while (*angle > 720.0f) {
        *angle -= 1440.0f;
    }
    while (*angle < -720.0f) {
        *angle += 1440.0f;
    }
}

void PID_Init(void) {
    /* 角度环PID初始化（外环）- 控制-720°到720° */
    angle_pid.Kp = 0.8f;     
    angle_pid.Ki = 0.01f;    
    angle_pid.Kd = 0.05f;   
    angle_pid.output_max = 200.0f;     
    angle_pid.integral_max = 300.0f;

    /* 速度环PID初始化（内环）*/
    speed_pid.Kp = 20.0f;  
    speed_pid.Ki = 0.3f;
    speed_pid.Kd = 0.08f;    
    speed_pid.output_max = 16000.0f;   
    speed_pid.integral_max = 16000.0f;
}


float PID_Calculate(PID_TypeDef *pid, float target, float actual) {
    pid->target_val = target;
    pid->actual_val = actual;
    pid->err = pid->target_val - pid->actual_val;

 
    pid->err_sum += pid->err;

    if (pid->err_sum > pid->integral_max) 
        pid->err_sum = pid->integral_max;
    if (pid->err_sum < -pid->integral_max) 
        pid->err_sum = -pid->integral_max;

   pid->output = pid->Kp * pid->err + 
                  pid->Ki * pid->err_sum + 
                  pid->Kd * (pid->err - pid->err_last);

 
    if (pid->output > pid->output_max) 
        pid->output = pid->output_max;
    if (pid->output < -pid->output_max) 
        pid->output = -pid->output_max;

    pid->err_last = pid->err; 

    return pid->output;
}

void Motor_Feedback_Update(Motor_Feedback_TypeDef *mf, uint8_t *rx_data) {
    mf->last_ecd = mf->ecd;
    mf->ecd = (uint16_t)((rx_data[0] << 8) | rx_data[1]);      
    mf->speed_rpm = (int16_t)((rx_data[2] << 8) | rx_data[3]);   
    mf->real_current = (int16_t)((rx_data[4] << 8) | rx_data[5]);
    mf->temperature = rx_data[6];                               


    int16_t diff = mf->ecd - mf->last_ecd;
    if (diff > 4096) {
        diff -= 8192;
        mf->round_cnt--;
    } else if (diff < -4096) {
        diff += 8192;
        mf->round_cnt++;
    }
    
  
    mf->total_angle = mf->round_cnt * 8192 + mf->ecd;
    
  
    mf->angle_deg = ((float)mf->total_angle / 8192.0f) * 360.0f;
    Angle_Normalization(&mf->angle_deg);
}

void CAN_Send_Current(int16_t current1, int16_t current2, int16_t current3, int16_t current4) {
    CAN_TxHeaderTypeDef tx_header;
    uint8_t tx_data[8];
    uint32_t tx_mailbox;
    
    tx_header.StdId = 0x200;        
    tx_header.ExtId = 0x00;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.IDE = CAN_ID_STD;
    tx_header.DLC = 8;
    tx_header.TransmitGlobalTime = DISABLE;
    
    // 填充数据（4个电机的电流值）
    tx_data[0] = (current1 >> 8) & 0xFF;   // 电机1电流高8位
    tx_data[1] = current1 & 0xFF;         // 电机1电流低8位
    tx_data[2] = (current2 >> 8) & 0xFF;  
    tx_data[3] = current2 & 0xFF;
    tx_data[4] = (current3 >> 8) & 0xFF;   
    tx_data[5] = current3 & 0xFF;
    tx_data[6] = (current4 >> 8) & 0xFF;
    tx_data[7] = current4 & 0xFF;
    
    HAL_CAN_AddTxMessage(&hcan, &tx_header, tx_data, &tx_mailbox);
}

void VOFA_ParseCommand(char* cmd) {
    if (strlen(cmd) < 2) return;

    char prefix = cmd[0];
    float value = atof(&cmd[1]);

    switch (prefix) {
        // 角度环PID调整
        case 'p': angle_pid.Kp = value; break;
        case 'i': angle_pid.Ki = value; break;  
        case 'd': angle_pid.Kd = value; break;
        
      //速度环
        case 'P': speed_pid.Kp = value; break;
        case 'I': speed_pid.Ki = value; break;
        case 'D': speed_pid.Kd = value; break;
        
      //目标
        case 'T': 
            if (value >= -720 && value <= 720) {
                angle_pid.target_val = value;
            }
            break;

        case 'R': 
            angle_pid.err_sum = 0;
            speed_pid.err_sum = 0;
            break;
        default:
            break;
    }
}

void VOFA_SendData(void) {
    
    printf("%.2f,%.2f,%.2f,%.2f,%.0f,%.3f,%.3f\n",
           angle_pid.target_val,      
           motor_fb.angle_deg,           
           speed_pid.target_val,        
           (float)motor_fb.speed_rpm,   
           (float)current_to_send,      
           angle_pid.Kp,               
           speed_pid.Kp);             
}
int _write(int file, char *ptr, int len) {
    HAL_UART_Transmit(&huart1, (uint8_t*)ptr, len, 1000);
    return len;
}


