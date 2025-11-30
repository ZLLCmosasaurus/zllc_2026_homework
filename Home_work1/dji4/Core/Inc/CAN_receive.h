#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "struct_typedef.h"

#define CHASSIS_CAN hcan


typedef enum
{
    CAN_CHASSIS_ALL_ID = 0x200,  
    CAN_3508_M1_ID = 0x201,     
    CAN_3508_M2_ID = 0x202,     
    CAN_3508_M3_ID = 0x203,     
    CAN_3508_M4_ID = 0x204,     
} can_3508_id_e;


typedef struct
{
    uint16_t ecd;          
    int16_t speed_rpm;   
    int16_t given_current; 
    uint8_t temperate;    
    int16_t last_ecd;    
} motor_measure_t;

//pid
typedef struct {
    float kp;         // 比例系数
    float ki;         // 积分系数
    float kd;         // 微分系数
    float ref;        // 目标值（如目标转速）
    float fdb;        // 反馈值（如实际转速）
    float err;        // 当前误差（ref - fdb）
    float err_last;   // 上一次误差
    float p_out;      // 比例项输出
    float i_out;      // 积分项输出
    float d_out;      // 微分项输出
    float i_max;      // 积分限幅（防止积分饱和）
    float out_max;    // 总输出限幅（电机最大电流）
    float output;     // PID总输出
} pid_struct_t;

// 限幅宏
#define LIMIT_MIN_MAX(x, min, max) (x) = (((x) <= (min)) ? (min) : (((x) >= (max)) ? (max) : (x)))
//pid函数声明
extern void PID_Motor1_Init(void);                  // PID初始化
extern void Set_Motor1_TargetSpeed(float rpm);      // 设置电机1目标转速
extern float pid_calc(pid_struct_t *pid, float ref, float fdb); // PID计算函数
extern pid_struct_t pid_motor1;                     // 电机1的PID实例

//extern void CAN_cmd_chassis_reset_ID(void);


extern void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);


extern const motor_measure_t *get_chassis_motor_measure_point(uint8_t i);

#endif


