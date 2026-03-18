#include "motor_control.h"

static float NormalizeAngle(float angle);

void MotorControl_Init(MotorControl_t *motor, uint8_t motor_id)
{
    motor->motor_id = motor_id;

    PID_Init(&motor->angle_pid, 10.0, 0.05, 0.2, 2000.0f, -2000.0f);

    PID_Init(&motor->speed_pid, 5.0, 0.1, 0.05, 16384.0f, -16384.0f);
    
    motor->feedback.rotor_angle = 0;
    motor->feedback.rotor_speed = 0;
    motor->feedback.torque_current = 0;
    motor->feedback.temperature = 0;
    motor->feedback.last_update_time = 0;
    
    motor->target_output_angle = 0.0f;
    motor->current_output_angle = 0.0f;
    motor->target_rotor_speed = 0.0f;
    motor->current_rotor_speed = 0.0f;
    
    motor->total_output_angle = 0.0f;
    motor->last_rotor_angle = 0.0f;
    motor->rotation_count = 0;
    
    motor->control_enable = 0;
}

void MotorControl_UpdateFeedback(MotorControl_t *motor, uint16_t rotor_angle, 
                                  int16_t rotor_speed, int16_t torque_current, 
                                  int8_t temperature)
{
    float rotor_angle_deg;
    float angle_diff;
    float output_angle_deg;
    
    motor->feedback.rotor_angle = rotor_angle;
    motor->feedback.rotor_speed = rotor_speed;
    motor->feedback.torque_current = torque_current;
    motor->feedback.temperature = temperature;
    
    rotor_angle_deg = (float)rotor_angle * 360.0 / 8191.0;
    
    angle_diff = rotor_angle_deg - motor->last_rotor_angle;

    if (angle_diff > 180.0)
    {
        motor->rotation_count--;
    } 
    else if (angle_diff < -180.0)
    {
        motor->rotation_count++;
    }
    
    float total_rotor_angle_deg = rotor_angle_deg + motor->rotation_count * 360.0;
    
    output_angle_deg = total_rotor_angle_deg / 19;
    motor->total_output_angle = output_angle_deg;
    
    motor->current_output_angle = NormalizeAngle(output_angle_deg);
    
    motor->current_rotor_speed = (float)rotor_speed;
    
    motor->last_rotor_angle = rotor_angle_deg;
}


void MotorControl_SetTargetAngle(MotorControl_t *motor, float target_angle_deg)
{
    motor->target_output_angle = motor->total_output_angle + target_angle_deg;
}

int16_t MotorControl_Calculate(MotorControl_t *motor)
{
    float angle_error;
    float speed_target;
    float current_output;
    
    if (motor->control_enable == 0) 
    {
        return 0;
    }
    
    angle_error = motor->target_output_angle - motor->total_output_angle;
    
    motor->angle_pid.target = 0.0f;
    speed_target = PID_Calculate(&motor->angle_pid, -angle_error);

    if (speed_target > 2000.0f) 
    {
        speed_target = 2000.0f;
    }
    else if (speed_target < -2000.0f)
    {
        speed_target = -2000.0f;
    }
    
    motor->target_rotor_speed = speed_target;

    PID_SetTarget(&motor->speed_pid, motor->target_rotor_speed);
    current_output = PID_Calculate(&motor->speed_pid, motor->current_rotor_speed);

    if (current_output > 16384.0f) 
    {
        current_output = 16384.0f;
    } 
    else if (current_output < -16384.0f) 
    {
        current_output = -16384.0f;
    }
    
    return (int16_t)current_output;
}

void MotorControl_Reset(MotorControl_t *motor)
{
    PID_Reset(&motor->angle_pid);
    PID_Reset(&motor->speed_pid);
    motor->target_output_angle = 0.0f;
    motor->target_rotor_speed = 0.0f;
    motor->rotation_count = 0;
    motor->total_output_angle = 0.0f;
    motor->last_rotor_angle = (float)motor->feedback.rotor_angle * 360.0f / 8191.0;
}

void MotorControl_SetEnable(MotorControl_t *motor, uint32_t enable)
{
    motor->control_enable = enable;

    if (enable == 0) 
    {
        MotorControl_Reset(motor);
    }
}


void MotorControl_RotateTwoRoundsForward(MotorControl_t *motor)
{
    motor->target_output_angle = motor->total_output_angle + 720.0;

    motor->control_enable = 1;
}

void MotorControl_RotateTwoRoundsReverse(MotorControl_t *motor)
{
    motor->target_output_angle = motor->total_output_angle - 720.0;

    motor->control_enable = 1;
}

/**
  * @brief  角度归一化到0-360度
  * @param  angle: 输入角度（度）
  * @retval 归一化后的角度（0-360度）
  */
static float NormalizeAngle(float angle)
{
    while (angle < 0.0f) 
    {
        angle += 360.0f;
    }
    while (angle >= 360.0f) 
    {
        angle -= 360.0f;
    }
    return angle;
}

/* USER CODE END 1 */

