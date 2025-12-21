#ifndef __BMI088_H
#define __BMI088_H

#include <stdbool.h>
#include <stdint.h>
#include "math.h"

#include "spi.h"
#include "gpio.h"
#include "bmi088_reg.h"

// BMI088 所使用的SPI
#define BMI088_SPI hspi1

// 加速度计片选引脚
#define BMI088_ACC_GPIOx GPIOA
#define BMI088_ACC_GPIOp GPIO_PIN_4

// 陀螺仪片选引脚
#define BMI088_GYRO_GPIOx     GPIOB
#define BMI088_GYRO_GPIOp     GPIO_PIN_0

#define BMI088_SPI_WRITE_CODE 0x7F
#define BMI088_SPI_READ_CODE  0x80

typedef struct {
    float x;
    float y;
    float z;
} Acc_Raw_Data_Typedef;

typedef struct {
    float roll;
    float pitch;
    float yaw;
} Gyro_Raw_Data_Typedef;

typedef struct {
    Acc_Raw_Data_Typedef acc_raw_data;
    float sensor_time;
    float temperature;
    bool enable_self_test;
} Acc_Data_Typedef;

typedef struct {
    Gyro_Raw_Data_Typedef gyro_raw_data;
    bool enable_self_test;
} Gyro_Data_Typedef;

typedef enum BMI088_Error {
    NO_ERR           = 0,
    ACC_CHIP_ID_ERR  = 0x01,
    ACC_DATA_ERR     = 0x02,
    GYRO_CHIP_ID_ERR = 0x04,
    GYRO_DATA_ERR    = 0x08,
} BMI088_Error;

typedef struct BMI088_Data_Typedef {
    Acc_Data_Typedef acc_data;
    BMI088_Error bmi088_err;
} BMI088_Data_Typedef;

extern bool BMI088_Init_Flag;

// 基础函数
void WriteDataToAcc(uint8_t addr, uint8_t data);
void WriteDataToGyro(uint8_t addr, uint8_t data);
void ReadSingleDataFromAcc(uint8_t addr, uint8_t *data);
void ReadSingleDataFromGyro(uint8_t addr, uint8_t *data);
void ReadMultiDataFromAcc(uint8_t addr, uint8_t len, uint8_t *data);
void ReadMultiDataFromGyro(uint8_t addr, uint8_t len, uint8_t *data);

// 初始化函数
BMI088_Error BMI088_Init();
void BMI088_Conf_Init();

// 功能函数
void ReadAccData(Acc_Raw_Data_Typedef *data);
void ReadGyroData(Gyro_Raw_Data_Typedef *data);
void ReadAccSensorTime(float *time);
void ReadAccTemperature(float *temp);

// 校验函数
BMI088_Error VerifyAccChipID(void);
BMI088_Error VerifyGyroChipID(void);
BMI088_Error VerifyAccSelfTest(void);
BMI088_Error VerifyGyroSelfTest(void);

#endif // !__BMI088_H