#ifndef __FILTERS_H
#define __FILTERS_H

#include <stdint.h>

// 一阶低通滤波器结构体
typedef struct {
    float alpha;        // 滤波系数 (0-1)，越小越平滑
    float prev_value;   // 上一次滤波后的值
    uint8_t initialized;
} LowPassFilter;

// 滑动平均滤波器结构体
typedef struct {
    float *buffer;      // 缓冲区
    uint32_t size;      // 缓冲区大小
    uint32_t index;     // 当前索引
    uint32_t count;     // 当前有效数据数量
    float sum;          // 当前总和
} MovingAverageFilter;

// 初始化低通滤波器
void LowPassFilter_Init(LowPassFilter *filter, float alpha, float initial_value);

// 低通滤波处理
float LowPassFilter_Process(LowPassFilter *filter, float input);

// 初始化滑动平均滤波器
void MovingAverageFilter_Init(MovingAverageFilter *filter, float *buffer, uint32_t size);

// 滑动平均滤波处理
float MovingAverageFilter_Process(MovingAverageFilter *filter, float input);

#endif