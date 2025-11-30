/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    can.c
 * @brief   This file provides code for the configuration
 *          of the CAN instances.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "can.h"
#include "usart.h"
#include <stdio.h>
/* USER CODE BEGIN 0 */

motor_feedback_t motor_feedback[4] = {{0}};
//uint8_t CAN_RxBuff[64];
// 连续角度跟踪（多圈），单位：度
float motor_continuous_angle[4] = {0.0f, 0.0f, 0.0f, 0.0f};
// 上一次原始角度（0..8191）用于计算跨越
uint16_t motor_last_angle_raw[4] = {0, 0, 0, 0};
// 标记是否已初始化连续角度
uint8_t motor_angle_inited[4] = {0, 0, 0, 0};
// UART发送状态标志
volatile uint8_t uart_tx_busy = 0;

/* USER CODE END 0 */

CAN_HandleTypeDef hcan;

/* CAN init function */
void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 4;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_2TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_3TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_5TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

    // 配置CAN过滤器
    CAN_FilterTypeDef can_filter;
    can_filter.FilterBank           = 0;
    can_filter.FilterMode           = CAN_FILTERMODE_IDMASK;
    can_filter.FilterScale          = CAN_FILTERSCALE_32BIT;
    can_filter.FilterIdHigh         = 0x0000;
    can_filter.FilterIdLow          = 0x0000;
    can_filter.FilterMaskIdHigh     = 0x0000;
    can_filter.FilterMaskIdLow      = 0x0000;
    can_filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    can_filter.FilterActivation     = ENABLE;
    can_filter.SlaveStartFilterBank = 14;

    if (HAL_CAN_ConfigFilter(&hcan, &can_filter) != HAL_OK) 
    {
        Error_Handler();
    }

    // 启动CAN
    if (HAL_CAN_Start(&hcan) != HAL_OK) 
    {
        Error_Handler();
    }

    // 激活CAN接收中断
    if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
        Error_Handler();
    }

  /* USER CODE END CAN_Init 2 */

}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**CAN GPIO Configuration
    PA11     ------> CAN_RX
    PA12     ------> CAN_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(USB_HP_CAN1_TX_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USB_HP_CAN1_TX_IRQn);
    HAL_NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();

    /**CAN GPIO Configuration
    PA11     ------> CAN_RX
    PA12     ------> CAN_TX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USB_HP_CAN1_TX_IRQn);
    HAL_NVIC_DisableIRQ(USB_LP_CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/**
 *  控制单个3508电机
 *  motor_id: 电机ID (1-4)
 *  current: 电流值 (-16384 到 16384)
 */
void can_dji_motor_control(uint8_t motor_id, int16_t current)
{
    CAN_TxHeaderTypeDef tx_header;
    uint8_t tx_data[8];
    uint32_t tx_mailbox;

    // 设置发送头
    tx_header.StdId = DJI_3508_CAN_ID;
    tx_header.ExtId = 0;
    tx_header.IDE   = CAN_ID_STD;
    tx_header.RTR   = CAN_RTR_DATA;
    tx_header.DLC   = 8;

    // 清零数据
    for (int i = 0; i < 8; i++) 
    {
        tx_data[i] = 0;
    }

    // 根据电机ID设置对应的电流值
    switch (motor_id) 
    {
        case 1:
            tx_data[0] = (current >> 8) & 0xFF;
            tx_data[1] = current & 0xFF;
            break;
        case 2:
            tx_data[2] = (current >> 8) & 0xFF;
            tx_data[3] = current & 0xFF;
            break;
        case 3:
            tx_data[4] = (current >> 8) & 0xFF;
            tx_data[5] = current & 0xFF;
            break;
        case 4:
            tx_data[6] = (current >> 8) & 0xFF;
            tx_data[7] = current & 0xFF;
            break;
        default:
            return;
    }

    // 发送CAN消息
    if (HAL_CAN_AddTxMessage(&hcan, &tx_header, tx_data, &tx_mailbox) != HAL_OK) 
    {
        // 错误处理
        Error_Handler();
    }
}

/**
 * @brief  Rx FIFO 0 message pending callback.
 * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
 *         the configuration information for the specified CAN.
 * @retval None
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rxHeader;
    uint8_t rx_data[8];

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, rx_data) == HAL_OK) 
    {
        can_receive_callback(&rxHeader, rx_data);
    }
}

/**
 *  同时控制四个DJI电机
 *  current1: 电机1电流值
 *  current2: 电机2电流值
 *  current3: 电机3电流值
 *  current4: 电机4电流值
 */
void can_dji_motor_control_multi(int16_t current1, int16_t current2, int16_t current3, int16_t current4)
{
    CAN_TxHeaderTypeDef tx_header;
    uint8_t tx_data[8];
    uint32_t tx_mailbox;

    // 设置发送头
    tx_header.StdId = DJI_3508_CAN_ID;
    tx_header.ExtId = 0;
    tx_header.IDE   = CAN_ID_STD;
    tx_header.RTR   = CAN_RTR_DATA;
    tx_header.DLC   = 8;

    // 填充数据
    tx_data[0] = (current1 >> 8) & 0xFF;
    tx_data[1] = current1 & 0xFF;
    tx_data[2] = (current2 >> 8) & 0xFF;
    tx_data[3] = current2 & 0xFF;
    tx_data[4] = (current3 >> 8) & 0xFF;
    tx_data[5] = current3 & 0xFF;
    tx_data[6] = (current4 >> 8) & 0xFF;
    tx_data[7] = current4 & 0xFF;

    // 发送CAN消息
    if (HAL_CAN_AddTxMessage(&hcan, &tx_header, tx_data, &tx_mailbox) != HAL_OK) 
    {
        // 错误处理
        Error_Handler();
    }
}

/**
 * CAN接收回调函数
 * rx_header: 接收消息头
 * rx_data: 接收数据
 */
void can_receive_callback(CAN_RxHeaderTypeDef *rx_header, uint8_t rx_data[])
{
    // 处理电机反馈数据
    if (rx_header->StdId >= DJI_3508_FEEDBACK_ID_START && rx_header->StdId <= (DJI_3508_FEEDBACK_ID_START + 3)) 
    {
        // 将 CAN ID 映射为 0-based 索引（期望 idx 为 0..3）
        uint8_t idx = (uint8_t)(rx_header->StdId - DJI_3508_FEEDBACK_ID_START);

        // 确保索引在有效范围内
        if (idx < 4) 
        {
            uint8_t motor_id = idx + 1; // 若需要人类可读的 1..4 ID
            // 解析反馈数据
            // DATA[0] = 转子机械角度高8位, DATA[1] = 角度低8位 (0~8191)
            // DATA[2,3] = 转子转速高/低8位 (signed 16)
            // DATA[4,5] = 实际转矩电流高/低8位 (signed 16)
            // DATA[6] = 电机温度 (1 byte), DATA[7] = Null
            uint16_t angle_u = ((uint16_t)rx_data[0] << 8) | (uint16_t)rx_data[1];
            // 角度范围为 0 ~ 8191 (13 位) -- 0 ~ 360 度
            angle_u &= 0x1FFFu;
            
            int16_t speed_i = (int16_t)(((uint16_t)rx_data[2] << 8) | (uint16_t)rx_data[3]);
            int16_t current_i = (int16_t)(((uint16_t)rx_data[4] << 8) | (uint16_t)rx_data[5]);
            uint8_t temp_u8 = rx_data[6];

            // 原始角度（度），浮点，用于连续角度计算
            float raw_angle_deg = ((float)angle_u) * 360.0f / 8191.0f;

            // 更新多圈连续角度：基于原始角度的跳变检测
            uint8_t idx = motor_id - 1;

            if (!motor_angle_inited[idx]) 
            {
                motor_continuous_angle[idx] = raw_angle_deg;
                motor_last_angle_raw[idx] = angle_u;
                motor_angle_inited[idx] = 1;
            } 
            else 
            {
                // 计算原始计数差（带环绕），angle_u 与 last 在 0..8191
                int32_t diff = (int32_t)angle_u - (int32_t)motor_last_angle_raw[idx];
                // 如果跳变过半圈（8192/2 = 4096），进行环绕修正
                if (diff > 4096) {
                    diff -= 8192;
                } else if (diff < -4096) {
                    diff += 8192;
                }
                // 将计数差转换为度数并累加
                float diff_deg = ((float)diff) * 360.0f / 8191.0f;
                motor_continuous_angle[idx] += diff_deg;
                motor_last_angle_raw[idx] = angle_u;
            }
        
            // 将当前单圈角度（四舍五入）保存到反馈结构（保留原有 int16_t 定义）
            motor_feedback[motor_id - 1].angle   = (int16_t)(raw_angle_deg + 0.5f);
            motor_feedback[motor_id - 1].speed   = speed_i;
            motor_feedback[motor_id - 1].current = current_i;
            motor_feedback[motor_id - 1].temp    = (int16_t)temp_u8;
            
             //通过UART发送电机反馈数据（CAN 接收数据）
            uint8_t uart_buffer[100];
            int len = snprintf((char*)uart_buffer, sizeof(uart_buffer), 
                               "M%d:Angle=%d,Speed=%d,Current=%d,Temp=%d\r\n", 
                               motor_id, 
                               motor_feedback[motor_id - 1].angle, 
                               motor_feedback[motor_id - 1].speed,
                               motor_feedback[motor_id - 1].current,
                               motor_feedback[motor_id - 1].temp);

            if (len > 0 && len < (int)sizeof(uart_buffer)) {
                // 避免在 CAN IRQ 中阻塞，使用非阻塞发送
                // 检查 UART 是否空闲；若空闲则发送，若忙则丢弃本次
                extern volatile uint8_t uart_tx_busy;

                if (!uart_tx_busy) {
                    static uint8_t can_tx_buf[128];
                    // 确保不溢出缓冲区
                    for (int i = 0; i < len; i++) can_tx_buf[i] = uart_buffer[i];
                    uart_tx_busy = 1;
                    HAL_UART_Transmit_IT(&huart1, can_tx_buf, (uint16_t)len);
                }
            }
        }
    }
}

/**
 * 相对旋转：使电机相对于当前连续角度旋转指定角度（单位度）
 * delta_angle: 正为正转（顺时针或定义的正向），负为反向
 * 该函数计算目标连续角度并发送一次速度命令（一次性）
 * 注意：这是一个单次命令函数，不包含闭环跟踪/循环直到完成的逻辑。
 */
void motor_rotate_by(uint8_t motor_id, float delta_angle, float speed_limit)
{
    if (motor_id < 1 || motor_id > 4) 
    {
        return;
    }

    uint8_t idx = motor_id - 1;
    // 如果还没初始化角度，就直接返回（等待首次反馈）
    if (!motor_angle_inited[idx]) 
    {
        return;
    }

    float current_continuous = motor_continuous_angle[idx];
    float target_continuous = current_continuous + delta_angle;
    float angle_diff = target_continuous - current_continuous; // 等于 delta_angle

    // 方向：根据角度差设置速度符号
    float target_speed = 0.0f;
    if (angle_diff > 0.0f) 
    {
        target_speed = speed_limit;
    } else if (angle_diff < 0.0f) 
    {
        target_speed = -speed_limit;
    }

    // 发送命令（使用 can_dji_motor_control，参数为电流，当前用速度映射到电流）
    can_dji_motor_control(motor_id, (int16_t)target_speed);
}

/**
 *  初始化PID控制器
 *  pid: PID控制器结构体指针
 *  kp: 比例系数
 *  ki: 积分系数
 *  kd: 微分系数
 */
void pid_init(pid_controller_t *pid, float kp, float ki, float kd)
{
    pid->kp         = kp;
    pid->ki         = ki;
    pid->kd         = kd;
    pid->target     = 0.0f;
    pid->current    = 0.0f;
    pid->error      = 0.0f;
    pid->last_error = 0.0f;
    pid->integral   = 0.0f;
    pid->output     = 0.0f;
}

/**
 * PID计算
 * pid: PID控制器结构体指针
 * target: 目标值
 * current: 当前值
 * dt: 采样时间间隔(s)
 * PID输出值
 */
float pid_calculate(pid_controller_t *pid, float target, float current, float dt)
{
    pid->target  = target;
    pid->current = current;
    pid->error   = target - current;
    // 防止 dt 非法（零或过小）导致除零或微分放大
    const float MIN_DT = 1e-4f;
    if (dt < MIN_DT) dt = MIN_DT;

    // 积分项（增量积分），并暂存以便在发生饱和时回退
    float integral_before = pid->integral;
    pid->integral += pid->error * dt;

    // 积分限幅，防止积分饱和
    const float INTEGRAL_LIMIT = 2000.0f; // 可调，增大以应对更大的稳态误差
    if (pid->integral > INTEGRAL_LIMIT) pid->integral = INTEGRAL_LIMIT;
    else if (pid->integral < -INTEGRAL_LIMIT) pid->integral = -INTEGRAL_LIMIT;

    // 微分项（使用保护后的 dt）
    float derivative = (pid->error - pid->last_error) / dt;

    // PID 输出（未限幅）
    float out = pid->kp * pid->error + pid->ki * pid->integral + pid->kd * derivative;

    // 输出限幅：匹配电机/控制接口允许的电流范围
    const float OUT_LIMIT = 16384.0f; // 电流命令的幅度上限
    if (out > OUT_LIMIT) {
        // 简单的反积分（undo last integration）以避免积分继续增大
        pid->integral = integral_before;
        out = OUT_LIMIT;
    } else if (out < -OUT_LIMIT) {
        pid->integral = integral_before;
        out = -OUT_LIMIT;
    }

    pid->output = out;
    pid->last_error = pid->error;

    return pid->output;
}

/**
 *  初始化串级PID控制器
 *  cascade_pid: 串级PID控制器结构体指针
 *  angle_kp: 角度环比例系数
 *  angle_ki: 角度环积分系数
 *  angle_kd: 角度环微分系数
 *  speed_kp: 速度环比例系数
 *  speed_ki: 速度环积分系数
 *  speed_kd: 速度环微分系数
 */
void cascade_pid_init(cascade_pid_controller_t *cascade_pid, float angle_kp, float angle_ki, float angle_kd, float speed_kp, float speed_ki, float speed_kd)
{
    // 初始化角度环PID控制器
    pid_init(&cascade_pid->angle_pid, angle_kp, angle_ki, angle_kd);
    
    // 初始化速度环PID控制器
    pid_init(&cascade_pid->speed_pid, speed_kp, speed_ki, speed_kd);
    
    // 初始化其他成员变量
    cascade_pid->target_angle = 0.0f;
    cascade_pid->target_speed = 0.0f;
    cascade_pid->current_angle = 0.0f;
    cascade_pid->current_speed = 0.0f;
}

/**
 * 串级PID计算
 * cascade_pid: 串级PID控制器结构体指针
 * target_angle: 目标角度
 * current_angle: 当前角度
 * current_speed: 当前速度
 * dt: 采样时间间隔(s)
 * 返回值: PID输出值
 */
float cascade_pid_calculate(cascade_pid_controller_t *cascade_pid, float target_angle, float current_angle, float current_speed, float dt)
{
    // 更新目标角度和当前角度
    cascade_pid->target_angle = target_angle;
    cascade_pid->current_angle = current_angle;
    cascade_pid->current_speed = current_speed;
    
    // 角度环计算，输出作为速度环的目标值
    cascade_pid->target_speed = pid_calculate(&cascade_pid->angle_pid, target_angle, current_angle, dt);
    
    // 速度环计算，输出作为最终的控制量
    float output_current = pid_calculate(&cascade_pid->speed_pid, cascade_pid->target_speed, current_speed, dt);
    
    return output_current;
}

/**
 * 电机旋转到指定角度
 * motor_id: 电机ID (1-4)
 * target_angle: 目标角度 (-720到720度)
 * speed_limit: 速度限制
 */
void motor_rotate_to_angle(uint8_t motor_id, float target_angle, float speed_limit)
{
    // 确保电机ID在有效范围内
    if (motor_id < 1 || motor_id > 4) 
    {
        return;
    }
    
    // 限制目标角度在-720到720度范围内
    if (target_angle > 720.0f) 
    {
        target_angle = 720.0f;
    }
    else if (target_angle < -720.0f) 
    {
        target_angle = -720.0f;
    }
    
    // 使用连续角度进行计算
    uint8_t idx = motor_id - 1;
    if (!motor_angle_inited[idx]) 
    {
        // 如果角度未初始化，直接返回
        return;
    }
    
    float current_angle = motor_continuous_angle[idx];
    
    // 计算角度差值
    float angle_diff = target_angle - current_angle;
    
    // 角度差值超过180度时，选择反向旋转路径以减少旋转角度
    if (angle_diff > 180.0f) 
    {
        angle_diff -= 360.0f;
    }
    else if (angle_diff < -180.0f) 
    {
        angle_diff += 360.0f;
    }
    
    // 根据角度差值确定旋转方向和速度，使用更平滑的控制策略
    float target_speed = 0.0f;
    if (angle_diff > 5.0f) 
    {  // 正向旋转
        // 使用比例控制，角度差值越大速度越快
        target_speed = (angle_diff > speed_limit) ? speed_limit : angle_diff;
    }
    else if (angle_diff < -5.0f) 
    {  // 反向旋转
        // 使用比例控制，角度差值越大速度越快
        target_speed = (angle_diff < -speed_limit) ? -speed_limit : angle_diff;
    }
    else
    {
        // 角度差值较小时，缓慢调整
        target_speed = angle_diff * 0.5f;  // 比例系数可调
    }
    
    // 控制电机旋转
    can_dji_motor_control(motor_id, (int16_t)target_speed);
}

/* USER CODE END 1 */
