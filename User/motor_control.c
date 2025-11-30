/* 电机控制源文件 motor_control.c */
#include "motor_control.h"
#include "math.h"

/* 电机对象数组 */
Motor_TypeDef motors[5];

/* CAN发送ID */
#define CAN_TX_ID 0x200

/* CAN接收ID范围 */
#define CAN_RX_ID_BASE 0x200
#define CAN_RX_ID_1 0x201
#define CAN_RX_ID_2 0x202
#define CAN_RX_ID_3 0x203
#define CAN_RX_ID_4 0x204

#define PID_CONTROL_PERIOD 10

int angle_flag=0;

float Angle_Diff(uint16_t current_raw, uint16_t last_raw)
{
    float angle_diff;
    float current_angle;
    
    // 计算角度差，处理0-8191边界跳变
    if (current_raw - last_raw > RAW_ANGLE_MAX / 2) {
        angle_diff = (float)current_raw - (float)last_raw - RAW_ANGLE_MAX;
    } else if (last_raw - current_raw > RAW_ANGLE_MAX / 2) {
        angle_diff = (float)current_raw - (float)last_raw + RAW_ANGLE_MAX;
    } else {
        angle_diff = (float)current_raw - (float)last_raw;
    }
    
    // 更新总角度（用于圈数计数）
    return angle_diff;
}

void PID_Init(PID_TypeDef *pid, float kp, float ki, float kd, float output_max)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->output = 0.0f;
    pid->output_max = output_max;
    pid->output_min = -output_max;
    pid->integral_max = output_max * 2.0f; // 积分限幅为输出限幅的2倍
    pid->last_time = HAL_GetTick();
}

/*
 * @retval 控制电流
 * @param 目标转速，真实转速
 */
float PID_Calculate(int motor_id,PID_TypeDef *pid, float target, float actual)
{
    uint32_t current_time = HAL_GetTick();
    float dt = (current_time - pid->last_time) / 1000.0f; // 转换为秒

    // 确保时间间隔为正数且合理
    if (dt <= 0 || dt > 0.1f)
    {
        dt = 0.01f; // 默认10ms
    }

    float error = target - actual;

    // 比例项
    float proportional = pid->kp * error;

    // 积分项（带积分限幅和抗积分饱和）

    pid->integral += error * dt;

    // 积分限幅
    if (pid->integral > pid->integral_max)
    {
        pid->integral = pid->integral_max;
    }
    else if (pid->integral < -pid->integral_max)
    {
        pid->integral = -pid->integral_max;
    }

    float integral = pid->ki * pid->integral;

    // 微分项
    float derivative = 0.0f;
    if (dt > 0)
    {
        derivative = pid->kd * (error - pid->prev_error) / dt;
    }

    // 计算输出
    pid->output = proportional + integral + derivative;

    // 输出限幅
    if (pid->output > pid->output_max)
    {
        pid->output = pid->output_max;
    }
    else if (pid->output < pid->output_min)
    {
        pid->output = pid->output_min;
    }

    // 更新状态
    pid->prev_error = error;
    pid->last_time = current_time;

    return pid->output;
}
/* 电机初始化函数 */
void Motor_Init(int motor_id)
{
    motors[motor_id].id = motor_id;
    motors[motor_id].target_speed = 0;
    motors[motor_id].actual_speed = 0;
		motors[motor_id].target_angle=0.0f;
	  motors[motor_id].actual_angle=0.0f;
    motors[motor_id].rotor_angle = 0;
		motors[motor_id].tot_rot = 0;
		motors[motor_id].last_angle = 0;
    motors[motor_id].torque_current = 0;
    motors[motor_id].temperature = 0;
    motors[motor_id].last_update_time = 0;
    PID_Init(&motors[motor_id].speed_pid, 3.4f, 0.5f, 0.10f, 8000.0f);
		PID_Init(&motors[motor_id].angle_pid, 10.0f, 0.05f, 0.80f, 1000.0f);
}
/* 设置电机转速函数 */
void Motor_Set_Speed(uint8_t motor_id, int16_t speed)
{
    if (motor_id < 1 || motor_id > 4)
    {
        return; // 电机ID错误
    }

		motors[motor_id].target_speed = speed;
    // 立即发送控制命令
    Motor_CAN_Tx_Handler(&hcan); 
}

/* 获取电机转速函数 */
int16_t Motor_Get_Speed(uint8_t motor_id)
{
    if (motor_id < 1 || motor_id > 4)
    {
        return 0; // 电机ID错误
    }

    return motors[motor_id].actual_speed;
}

int16_t Motor_Get_Target_Speed(uint8_t motor_id)
{
    if (motor_id < 1 || motor_id > 4)
    {
        return 0; // 电机ID错误
    }

    return motors[motor_id].target_speed;
}

void Motor_Set_Angle(int motor_id,float angle)
{
	motors[motor_id].target_angle=angle;
}

float Motor_Get_Angle(int motor_id)
{
	return motors[motor_id].actual_angle;
}

float Motor_Get_Target_Angle(int motor_id)
{
	return motors[motor_id].target_angle;
}

void Motor_Set_Last_Angle(int motor_id)
{
	motors[motor_id].last_angle = motors[motor_id].rotor_angle;
}

void Motor_PID_Update(uint8_t motor_id)
{
    static uint32_t last_pid_time = 0;
    uint32_t current_time = HAL_GetTick();

    // 检查是否到达PID控制周期
    if (current_time - last_pid_time < PID_CONTROL_PERIOD)
    {
        return;
    }
    float pid_output = PID_Calculate(motor_id,&motors[motor_id].speed_pid, (float)motors[motor_id].target_speed, (float)motors[motor_id].actual_speed);
    int16_t current_control = (int16_t)pid_output;

    // 限制电流范围（根据电机规格调整）
    if (current_control > 10000)
        current_control = 10000;
    if (current_control < -10000)
        current_control = -10000;
    last_pid_time = current_time;
    // 更新目标速度（实际发送的是电流值）
    motors[motor_id].control_current = current_control;
		Motor_CAN_Tx_Handler(&hcan); 
}

void Motor_Muti_PID_Update(uint8_t motor_id)
{
    static uint32_t last_pid_time = 0;
    uint32_t current_time = HAL_GetTick();

    // 检查是否到达PID控制周期
    if (current_time - last_pid_time < PID_CONTROL_PERIOD)
    {
        return;
    }
		
    float current_output = PID_Calculate(motor_id,&motors[motor_id].angle_pid, (float)motors[motor_id].target_angle, (float)motors[motor_id].actual_angle);
    
    float target_spd = PID_Calculate(motor_id,&motors[motor_id].speed_pid, motors[motor_id].target_speed, motors[motor_id].actual_speed);
		int16_t current_control = (int16_t)current_output;
    // 限制电流范围（根据电机规格调整）
    if (current_control > 10000)
        current_control = 10000;
    if (current_control < -10000)
        current_control = -10000;
    last_pid_time = current_time;
    // 更新目标速度（实际发送的是电流值）
    motors[motor_id].control_current = current_control;
		Motor_CAN_Tx_Handler(&hcan); 
}

/* CAN发送处理函数 - 发送电机控制命令 */
void Motor_CAN_Tx_Handler(CAN_HandleTypeDef *hcan)
{
    CAN_TxHeaderTypeDef tx_header;
    uint8_t tx_data[8];
    uint32_t tx_mailbox;

    /* 配置CAN发送头 */
    tx_header.StdId = CAN_TX_ID;  // 标准ID: 0x200
    tx_header.ExtId = 0x00;       // 扩展ID: 不使用
    tx_header.IDE = CAN_ID_STD;   // 标准帧
    tx_header.RTR = CAN_RTR_DATA; // 数据帧
    tx_header.DLC = 8;            // 数据长度: 8字节
    tx_header.TransmitGlobalTime = DISABLE;

    /* 填充数据域 - 同时控制4个电机 */
    tx_data[0] = (motors[1].control_current >> 8) & 0xFF; // 电机1电流高8位
    tx_data[1] = motors[2].control_current & 0xFF;        // 电机1电流低8位
    tx_data[2] = (motors[2].control_current >> 8) & 0xFF; // 电机2电流高8位
    tx_data[3] = motors[2].control_current & 0xFF;        // 电机2电流低8位
    tx_data[4] = (motors[3].control_current >> 8) & 0xFF; // 电机3电流高8位
    tx_data[5] = motors[3].control_current & 0xFF;        // 电机3电流低8位
    tx_data[6] = (motors[4].control_current >> 8) & 0xFF; // 电机4电流高8位
    tx_data[7] = motors[4].control_current & 0xFF;        // 电机4电流低8位

    /* 发送CAN帧 */
    if (HAL_CAN_AddTxMessage(hcan, &tx_header, tx_data, &tx_mailbox) != HAL_OK)
    {
        // 发送错误处理
        Error_Handler();
    }
}

/* CAN接收处理函数 - 解析电机反馈数据 */
void Motor_CAN_Rx_Handler(CAN_HandleTypeDef *hcan, CAN_RxHeaderTypeDef *rx_header, uint8_t *rx_data)
{
    uint8_t motor_id;

    /* 检查是否为电机反馈帧 */
    if (rx_header->IDE == CAN_ID_STD && rx_header->StdId >= CAN_RX_ID_1 && rx_header->StdId <= CAN_RX_ID_4)
    {
        motor_id = rx_header->StdId - CAN_RX_ID_BASE;
				motors[motor_id].last_angle = motors[motor_id].rotor_angle;
        if (motor_id >= 1 && motor_id <= 4)
        {
            motors[motor_id].rotor_angle = (rx_data[0] << 8) | rx_data[1];    // 转子机械角度
            motors[motor_id].actual_speed = (rx_data[2] << 8) | rx_data[3];   // 转子转速
            motors[motor_id].torque_current = (rx_data[4] << 8) | rx_data[5]; // 转矩电流
            motors[motor_id].temperature = rx_data[6];                        // 电机温度
            motors[motor_id].last_update_time = HAL_GetTick();                // 更新时间戳
        }
				if (motors[motor_id].zero_angle == 0)
				{
					motors[motor_id].zero_angle = motors[motor_id].rotor_angle;
					motors[motor_id].actual_angle=1;
					motors[motor_id].tot_rot=0;
				}
				uint16_t now_angle=0;
				
				if (motors[motor_id].last_angle - motors[motor_id].rotor_angle >= RAW_ANGLE_MAX / 2)
				{
					//motors[motor_id].actual_angle = motors[motor_id].rotor_angle + RAW_ANGLE_MAX - motors[motor_id].zero_angle;
					angle_flag=1;
					motors[motor_id].tot_rot++;
				}
				else if (motors[motor_id].rotor_angle - motors[motor_id].last_angle >= RAW_ANGLE_MAX / 2)
				{
					//motors[motor_id].actual_angle = motors[motor_id].rotor_angle - RAW_ANGLE_MAX - motors[motor_id].zero_angle;
					angle_flag=2;
					motors[motor_id].tot_rot--;
				}
				else 
				{
					//motors[motor_id].actual_angle = motors[motor_id].rotor_angle  - motors[motor_id].zero_angle;
					angle_flag=3;
				}
				motors[motor_id].actual_angle = motors[motor_id].tot_rot * RAW_ANGLE_MAX + motors[motor_id].rotor_angle;
        motors[motor_id].raw_angle = motors[motor_id].rotor_angle; 
				
				
    }
}
