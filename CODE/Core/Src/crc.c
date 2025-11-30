#include "crc.h"

uint8_t crc8_table[256];

// 预先计算CRC8查找表
void crc8_init_table()
{
    uint16_t i, j;
    uint8_t crc;

    for (i = 0; i < 256; i++) {
        crc = (uint8_t)i;
        for (j = 0; j < 8; j++) {
            if (crc & 0x80) {
                // 这里的处理逻辑与直接计算法类似，但更简洁
                crc = (crc << 1) ^ CRC8_POLYNOMIAL;
            } else {
                crc <<= 1;
            }
        }
        crc8_table[i] = crc;
    }
}

// 使用查找表计算CRC8
uint8_t crc8_table_driven(const uint8_t *data, size_t length)
{
    uint8_t crc = CRC8_INITIAL_VALUE;
    size_t i;

    // 假设 crc8_init_table() 已经在程序开始时调用
    for (i = 0; i < length; i++) {
        // 使用当前CRC的高位（前一个字节计算的结果）和下一个数据字节来查找表
        // 索引 = 当前CRC ^ 当前数据字节
        crc = crc8_table[crc ^ data[i]];
    }

    // 假设不需要额外的 XOR_OUT 操作
    return crc;
}