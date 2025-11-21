#include "wheel.h"
#include "pid.h"
#include "math.h"
#include "stdlib.h"

uint64_t wheel_timestamp;
uint64_t WHEEL_PULSE[4]   = {0, 0, 0, 0};
float wheel_target_rpm[4] = {0, 0, 0, 0};
PID_PWM_TypeDef wheel_pid[4];
float wheel_actual_rpm[4]         = {0, 0, 0, 0};
uint8_t wheel_actual_direction[4] = {STOP, STOP, STOP, STOP};
float max_speed                   = 0.5f;

void Wheel_Init(void)
{
    // 编码器初始化
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    __HAL_TIM_SetCounter(&htim3, 32767);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
    __HAL_TIM_SetCounter(&htim4, 32767);

    // PWM定时器启动
    HAL_TIM_PWM_Start(WHEEL_LF_PWM_TIM, WHEEL_LF_PWM_CHANNEL);
    HAL_TIM_PWM_Start(WHEEL_RF_PWM_TIM, WHEEL_RF_PWM_CHANNEL);
    HAL_TIM_PWM_Start(WHEEL_LB_PWM_TIM, WHEEL_LB_PWM_CHANNEL);
    HAL_TIM_PWM_Start(WHEEL_RB_PWM_TIM, WHEEL_RB_PWM_CHANNEL);

    // 设置状态
    SetWheelDirection(1, STOP);
    SetWheelDirection(2, STOP);
    SetWheelDirection(3, STOP);
    SetWheelDirection(4, STOP);

    // 设置初速
    SetWheelTargetRPM(1, 0.0f);
    SetWheelTargetRPM(2, 0.0f);
    SetWheelTargetRPM(3, 0.0f);
    SetWheelTargetRPM(4, 0.0f);

    PID_Init(&wheel_pid[0], 0.2f, 0.05f, 0.005f, 90.0f, 0.0f);
    PID_Init(&wheel_pid[1], 0.2f, 0.05f, 0.005f, 90.0f, 0.0f);
    PID_Init(&wheel_pid[2], 0.2f, 0.05f, 0.005f, 90.0f, 0.0f);
    PID_Init(&wheel_pid[3], 0.2f, 0.05f, 0.005f, 90.0f, 0.0f);

    wheel_timestamp = HAL_GetTick();
}

float GetWheelActualRPM(uint8_t wheel)
{
    return wheel_actual_rpm[wheel - 1];
}

void SetWheelTargetRPM(uint8_t wheel, float rpm)
{
    wheel_target_rpm[wheel - 1] = rpm;
}

void UpdateWheelRPM()
{
    uint64_t ms     = HAL_GetTick() - wheel_timestamp;
    wheel_timestamp = HAL_GetTick();

    WHEEL_PULSE[WHEEL_LF - 1] = abs(__HAL_TIM_GET_COUNTER(&htim3) - 32767);
    __HAL_TIM_SetCounter(&htim3, 32767);

    WHEEL_PULSE[WHEEL_RB - 1] = abs(__HAL_TIM_GET_COUNTER(&htim4) - 32767);
    __HAL_TIM_SetCounter(&htim4, 32767);

    for (int i = 0; i < 4; i++) {
        wheel_actual_rpm[i] = WHEEL_PULSE[i] * 60 * 1000 / 1320 / ms;
        WHEEL_PULSE[i]      = 0;
    }

    float pwm1 = PID_Calculate(&wheel_pid[WHEEL_LF - 1], wheel_target_rpm[WHEEL_LF - 1], wheel_actual_rpm[WHEEL_LF - 1]);
    __HAL_TIM_SET_COMPARE(WHEEL_LF_PWM_TIM, WHEEL_LF_PWM_CHANNEL, (uint16_t)(100 * pwm1));
    float pwm2 = PID_Calculate(&wheel_pid[WHEEL_RF - 1], wheel_target_rpm[WHEEL_RF - 1], wheel_actual_rpm[WHEEL_RF - 1]);
    __HAL_TIM_SET_COMPARE(WHEEL_RF_PWM_TIM, WHEEL_RF_PWM_CHANNEL, (uint16_t)(100 * pwm2));
    float pwm3 = PID_Calculate(&wheel_pid[WHEEL_LB - 1], wheel_target_rpm[WHEEL_LB - 1], wheel_actual_rpm[WHEEL_LB - 1]);
    __HAL_TIM_SET_COMPARE(WHEEL_LB_PWM_TIM, WHEEL_LB_PWM_CHANNEL, (uint16_t)(100 * pwm3));
    float pwm4 = PID_Calculate(&wheel_pid[WHEEL_RB - 1], wheel_target_rpm[WHEEL_RB - 1], wheel_actual_rpm[WHEEL_RB - 1]);
    __HAL_TIM_SET_COMPARE(WHEEL_RB_PWM_TIM, WHEEL_RB_PWM_CHANNEL, (uint16_t)(100 * pwm4));
}

void SetWheelDirection(uint8_t wheel, uint8_t direction)
{
    switch (wheel) {
        case WHEEL_LF:
            switch (direction) {
                case STOP:
                    HAL_GPIO_WritePin(WHEEL_LF_IN1_GPIO, WHEEL_LF_IN1, RESET);
                    HAL_GPIO_WritePin(WHEEL_LF_IN2_GPIO, WHEEL_LF_IN2, RESET);
                    break;
                case CLOCKWISE:
                    HAL_GPIO_WritePin(WHEEL_LF_IN1_GPIO, WHEEL_LF_IN1, RESET);
                    HAL_GPIO_WritePin(WHEEL_LF_IN2_GPIO, WHEEL_LF_IN2, SET);
                    break;
                case COUNTERCLOCKWISE:
                    HAL_GPIO_WritePin(WHEEL_LF_IN1_GPIO, WHEEL_LF_IN1, SET);
                    HAL_GPIO_WritePin(WHEEL_LF_IN2_GPIO, WHEEL_LF_IN2, RESET);
                    break;
                case PLUSE:
                    HAL_GPIO_WritePin(WHEEL_LF_IN1_GPIO, WHEEL_LF_IN1, SET);
                    HAL_GPIO_WritePin(WHEEL_LF_IN2_GPIO, WHEEL_LF_IN2, SET);
                    break;
                default:
                    return;
            }
            break;
        case WHEEL_RF:
            switch (direction) {
                case STOP:
                    HAL_GPIO_WritePin(WHEEL_RF_IN1_GPIO, WHEEL_RF_IN1, RESET);
                    HAL_GPIO_WritePin(WHEEL_RF_IN2_GPIO, WHEEL_RF_IN2, RESET);
                    break;
                case CLOCKWISE:
                    HAL_GPIO_WritePin(WHEEL_RF_IN1_GPIO, WHEEL_RF_IN1, SET);
                    HAL_GPIO_WritePin(WHEEL_RF_IN2_GPIO, WHEEL_RF_IN2, RESET);
                    break;
                case COUNTERCLOCKWISE:
                    HAL_GPIO_WritePin(WHEEL_RF_IN1_GPIO, WHEEL_RF_IN1, RESET);
                    HAL_GPIO_WritePin(WHEEL_RF_IN2_GPIO, WHEEL_RF_IN2, SET);
                    break;
                case PLUSE:
                    HAL_GPIO_WritePin(WHEEL_RF_IN1_GPIO, WHEEL_RF_IN1, SET);
                    HAL_GPIO_WritePin(WHEEL_RF_IN2_GPIO, WHEEL_RF_IN2, SET);
                    break;
                default:
                    return;
            }
            break;
        case WHEEL_LB:
            switch (direction) {
                case STOP:
                    HAL_GPIO_WritePin(WHEEL_LB_IN1_GPIO, WHEEL_LB_IN1, RESET);
                    HAL_GPIO_WritePin(WHEEL_LB_IN2_GPIO, WHEEL_LB_IN2, RESET);
                    break;
                case CLOCKWISE:
                    HAL_GPIO_WritePin(WHEEL_LB_IN1_GPIO, WHEEL_LB_IN1, RESET);
                    HAL_GPIO_WritePin(WHEEL_LB_IN2_GPIO, WHEEL_LB_IN2, SET);
                    break;
                case COUNTERCLOCKWISE:
                    HAL_GPIO_WritePin(WHEEL_LB_IN1_GPIO, WHEEL_LB_IN1, SET);
                    HAL_GPIO_WritePin(WHEEL_LB_IN2_GPIO, WHEEL_LB_IN2, RESET);
                    break;
                case PLUSE:
                    HAL_GPIO_WritePin(WHEEL_LB_IN1_GPIO, WHEEL_LB_IN1, SET);
                    HAL_GPIO_WritePin(WHEEL_LB_IN2_GPIO, WHEEL_LB_IN2, SET);
                    break;
                default:
                    return;
            }
            break;
        case WHEEL_RB:
            switch (direction) {
                case STOP:
                    HAL_GPIO_WritePin(WHEEL_RB_IN1_GPIO, WHEEL_RB_IN1, RESET);
                    HAL_GPIO_WritePin(WHEEL_RB_IN2_GPIO, WHEEL_RB_IN2, RESET);
                    break;
                case CLOCKWISE:
                    HAL_GPIO_WritePin(WHEEL_RB_IN1_GPIO, WHEEL_RB_IN1, RESET);
                    HAL_GPIO_WritePin(WHEEL_RB_IN2_GPIO, WHEEL_RB_IN2, SET);
                    break;
                case COUNTERCLOCKWISE:
                    HAL_GPIO_WritePin(WHEEL_RB_IN1_GPIO, WHEEL_RB_IN1, SET);
                    HAL_GPIO_WritePin(WHEEL_RB_IN2_GPIO, WHEEL_RB_IN2, RESET);
                    break;
                case PLUSE:
                    HAL_GPIO_WritePin(WHEEL_RB_IN1_GPIO, WHEEL_RB_IN1, SET);
                    HAL_GPIO_WritePin(WHEEL_RB_IN2_GPIO, WHEEL_RB_IN2, SET);
                    break;
                default:
                    return;
            }
            break;
        default:
            return;
    }
    wheel_actual_direction[wheel - 1] = direction;
}

uint8_t GetWheelDirection(uint8_t wheel)
{
    return wheel_actual_direction[wheel - 1];
}

/**
 * @brief VA轮 = Vx+Vy-Vz*(H/2+W/2)
 *        VB轮 = Vx-Vy-Vz*(H/2+W/2)
 *        VC轮 = Vx+Vy+Vz*(H/2+W/2)
 *        VD轮 = Vx-Vy+Vz*(H/2+W/2)
 *        参数说明：
 *        VABCD轮-> 麦轮A、B、C、D 的线速度，单位m/s。
 *        W-> 轮距，机器人左右麦轮的距离，单位：m。
 *        H-> 轴距，机器人前后麦轮的距离，单位：m。
 *
 * @param Vx 机器人前后移动速度，前进为正，单位：m/s。
 * @param Vy 机器人左右移动速度，左移为正，单位：m/s。
 * @param Vz 机器人绕 O 点旋转速度，逆时针为正，单位：rad/s
 */
void CarMove(float Vx, float Vy, float Vz)
{
    float Va = Vx + Vy - Vz * T;
    float Vb = Vx - Vy - Vz * T;
    float Vc = Vx + Vy + Vz * T;
    float Vd = Vx - Vy + Vz * T;

    if (Va < 0) {
        SetWheelDirection(WHEEL_LB, CLOCKWISE);
    } else if (Va > 0) {
        SetWheelDirection(WHEEL_LB, COUNTERCLOCKWISE);
    } else {
        SetWheelDirection(WHEEL_LB, STOP);
    }
    SetWheelTargetRPM(WHEEL_LB, fabsf(Va) * 60 / (RADIUS * 2 * M_PI));

    if (Vb < 0) {
        SetWheelDirection(WHEEL_LF, CLOCKWISE);
    } else if (Vb > 0) {
        SetWheelDirection(WHEEL_LF, COUNTERCLOCKWISE);
    } else {
        SetWheelDirection(WHEEL_LF, STOP);
    }
    SetWheelTargetRPM(WHEEL_LF, fabsf(Vb) / (RADIUS * 2 * M_PI) * 60);

    if (Vc < 0) {
        SetWheelDirection(WHEEL_RF, CLOCKWISE);
    } else if (Vc > 0) {
        SetWheelDirection(WHEEL_RF, COUNTERCLOCKWISE);
    } else {
        SetWheelDirection(WHEEL_RF, STOP);
    }
    SetWheelTargetRPM(WHEEL_RF, fabsf(Vc) / (RADIUS * 2 * M_PI) * 60);

    if (Vd < 0) {
        SetWheelDirection(WHEEL_RB, CLOCKWISE);
    } else if (Vd > 0) {
        SetWheelDirection(WHEEL_RB, COUNTERCLOCKWISE);
    } else {
        SetWheelDirection(WHEEL_RB, STOP);
    }
    SetWheelTargetRPM(WHEEL_RB, fabsf(Vd) / (RADIUS * 2 * M_PI) * 60);
}

void CarSpin(uint8_t direction)
{
    if (direction == CLOCKWISE) {
        CarMove(0, 0, -max_speed);
    } else if (direction == COUNTERCLOCKWISE) {
        CarMove(0, 0, max_speed);
    }
}

void CarGoStraight(float angle)
{
    CarMove(max_speed * cosf(angle), max_speed * sinf(angle), 0);
}
