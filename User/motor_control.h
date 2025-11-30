/* 电机控制头文件 motor_control.h */
#ifndef __MOTOR_CONTROL_H
#define __MOTOR_CONTROL_H

#include "main.h"
#include "can.h"

#define MAX_ANGLE 720.0f           // 最大角度 ±720度
#define RAW_ANGLE_MAX 8192.0f      // 原始角度最大值
#define RAW_TO_DEGREE (360.0f / RAW_ANGLE_MAX)  // 原始角度转度数的系数

typedef struct {
    float kp;           // 比例系数
    float ki;           // 积分系数  
    float kd;           // 微分系数
    float integral;     // 积分项
    float prev_error;   // 上一次误差
    float output;       // 输出值
    float output_max;   // 输出上限
    float output_min;   // 输出下限
    float integral_max; // 积分限幅
    uint32_t last_time; // 上次计算时间
} PID_TypeDef;

/* 电机参数结构体 */
typedef struct {
    uint16_t id;
	
    int16_t target_speed;
    int16_t actual_speed;
	
		float target_angle;
		float actual_angle;
	
		uint16_t zero_angle;
    
    uint16_t rotor_angle; 
		uint16_t raw_angle;
		uint16_t last_angle;
		int16_t tot_rot;
	
		uint16_t control_current;
	
    int16_t torque_current;
    uint8_t temperature;
    uint32_t last_update_time;
    PID_TypeDef speed_pid;
		PID_TypeDef angle_pid;
} Motor_TypeDef;

/* 函数声明 */
void Motor_Init(int motor_id);
void Motor_Set_Speed(uint8_t motor_id, int16_t speed);
int16_t Motor_Get_Speed(uint8_t motor_id);
int16_t Motor_Get_Target_Speed(uint8_t motor_id);

void Motor_Set_Angle(int motor_id,float angle);
void Motor_Set_Last_Angle(int motor_id);
float Motor_Get_Angle(int motor_id);
float Motor_Get_Target_Angle(int motor_id);

void Motor_CAN_Tx_Handler(CAN_HandleTypeDef *hcan);
void Motor_CAN_Rx_Handler(CAN_HandleTypeDef *hcan, CAN_RxHeaderTypeDef *rx_header, uint8_t *rx_data);
void Motor_PID_Update(uint8_t motor_id);
void PID_Init(PID_TypeDef *pid, float kp, float ki, float kd, float output_max);
float PID_Calculate(int motor_id,PID_TypeDef *pid, float target, float actual);
float Process_Angle(uint16_t current_raw, uint16_t last_raw, int32_t *total_rotation);
void Motor_Muti_PID_Update(uint8_t motor_id);

#endif
