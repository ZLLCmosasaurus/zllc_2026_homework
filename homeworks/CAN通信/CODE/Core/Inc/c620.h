#ifndef __C620_H
#define __C620_H

#include "stm32f1xx.h"

#define MAX_SAFE_CURRENT

/**
 * @brief C620电调返回报文数据
 *
 * @param id 电调ID 范围为1-8
 * @param angle 转子机械角度
 *      0\~8191，对应0\~360度
 * @param speed 转子转速 单位RPM
 * @param current 实际转矩电流 -20\~0\~20A
 */
typedef struct
{
    uint8_t id;
    float angle;
    int16_t speed;
    float current;
    uint8_t temperature;
} C620_Motor_Status_TypeDef;

/**
 * @brief 将CAN总线接受到的数据解析到`C620_Motor_Status_TypeDef`中
 *
 * @param status `C620_Motor_Status_TypeDef`结构体指针
 * @param stdId 收到的CAN报文的标识符
 * @param data 收到的数据
 */
void C620_Motor_Status_Init(C620_Motor_Status_TypeDef *status, uint32_t stdId, const uint8_t *data);

/**
 * @brief 向C620电调发送报文数据
 *
 * @param id 要发送到的电调
 * @param current 实际转矩电流 -20\~0\~20A
 */
typedef struct
{
    uint8_t id;
    float current;
} C620_Motor_Control_Typedef;

/**
 * @brief 将`C620_Motor_Control_Typedef`结构体中的数据解析到要发送的数据包的对应位置上
 *
 * @param control `C620_Motor_Control_Typedef`结构体指针
 * @param data 要发送的数据
 */
void C620_Motor_Control_Init(C620_Motor_Control_Typedef *control, uint8_t *data);

#endif // !__C620_H