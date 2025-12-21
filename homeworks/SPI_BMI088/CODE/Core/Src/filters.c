// filters.c
#include "filters.h"
#include <string.h>

void LowPassFilter_Init(LowPassFilter *filter, float alpha, float initial_value) {
    filter->alpha = alpha;
    filter->prev_value = initial_value;
    filter->initialized = 1;
}

float LowPassFilter_Process(LowPassFilter *filter, float input) {
    if (!filter->initialized) {
        filter->prev_value = input;
        filter->initialized = 1;
        return input;
    }
    
    filter->prev_value = filter->alpha * input + (1.0f - filter->alpha) * filter->prev_value;
    return filter->prev_value;
}

void MovingAverageFilter_Init(MovingAverageFilter *filter, float *buffer, uint32_t size) {
    filter->buffer = buffer;
    filter->size = size;
    filter->index = 0;
    filter->count = 0;
    filter->sum = 0.0f;
    memset(buffer, 0, size * sizeof(float));
}

float MovingAverageFilter_Process(MovingAverageFilter *filter, float input) {
    // 减去即将被覆盖的值
    if (filter->count == filter->size) {
        filter->sum -= filter->buffer[filter->index];
    } else {
        filter->count++;
    }
    
    // 添加新值
    filter->buffer[filter->index] = input;
    filter->sum += input;
    
    // 更新索引
    filter->index++;
    if (filter->index >= filter->size) {
        filter->index = 0;
    }
    
    // 计算平均值
    return filter->sum / filter->count;
}