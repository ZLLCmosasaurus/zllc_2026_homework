#ifndef __CRC_H
#define __CRC_H

#include "main.h"

// 定义CRC8多项式（Polynomial）。
// 例如：0x1D (x^8 + x^5 + x^4 + x^3 + x^2 + 1) 是一个常见的多项式。
#define CRC8_POLYNOMIAL 0x1D

// 定义初始值（Initial value）。
// 常见的初始值有 0x00 或 0xFF。
// 某些协议可能要求在计算前对数据或结果进行异或操作（XOR）。
#define CRC8_INITIAL_VALUE 0x00

void crc8_init_table();
uint8_t crc8_table_driven(const uint8_t *data, size_t length);

#endif // !__CRC_H