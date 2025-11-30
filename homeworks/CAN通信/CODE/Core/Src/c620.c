#include "c620.h"
#include <math.h>

float convert_raw_to_angle(uint16_t raw)
{
    const float SCALE_FACTOR = 360.0f / 8191.0f;
    return raw * SCALE_FACTOR;
}

uint16_t convert_angle_to_raw(float angle)
{
    const float SCALE_FACTOR = 8191.0f / 360.0f;
    return (int16_t)roundf(angle * SCALE_FACTOR);
}

float convert_raw_to_amps(int16_t raw)
{
    const float SCALE_FACTOR = 20.0f / 16384.0f;
    return raw * SCALE_FACTOR;
}

int16_t convert_amps_to_raw(float current_amps)
{
    const float SCALE_FACTOR = 16384.0f / 20.0f;
    return (int16_t)roundf(current_amps * SCALE_FACTOR);
}

void C620_Motor_Status_Init(C620_Motor_Status_TypeDef *status, uint32_t stdId, const uint8_t *data)
{
    status->id          = stdId & 0x00F;
    status->angle       = convert_raw_to_angle((data[0] << 8) | data[1]);
    status->speed       = (data[2] << 8) | data[3];
    status->current     = convert_raw_to_amps((data[4] << 8) | data[5]);
    status->temperature = data[6];
}

void C620_Motor_Control_Init(C620_Motor_Control_Typedef *control, uint8_t *data)
{
    uint8_t start   = (control->id > 4) ? ((control->id - 4) * 2 - 1) : (control->id * 2 - 2);
    uint16_t raw    = convert_amps_to_raw(control->current);
    data[start]     = (raw & 0xFF00) >> 8;
    data[start + 1] = raw & 0x00FF;
}
