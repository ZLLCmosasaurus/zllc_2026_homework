# SPI的使用 与 BMI088解算姿态

## SPI 介绍

`SPI`是一种高速、全双工、同步的通信总线，全称`Serial Peripheral Interface`。一般是一主多从的形式，时钟由主设备提供，数据在上升沿或下降沿进行采样。

### 信号线

- **MOSI**（主设备输出/从设备输入）：主设备发送数据，从设备接收数据。

- **MISO**（主设备输入/从设备输出）：主设备接收数据，从设备发送数据。

- **SCLK**（串行时钟）：由主设备产生的时钟信号。

- **CS/SS**（片选信号）：由主设备控制，用于选择特定的从设备。

  一些设备是片选信号为低电平时启用、一些反之。许要根据实际情况配置软件高低电平或者在硬件上加上非门。

### 通信模式

- **Mode 0**：`CPOL`=0，`CPHA`=0，时钟空闲状态为低电平，数据在上升沿采样。
- **Mode 1**：`CPOL`=0，`CPHA`=1，时钟空闲状态为低电平，数据在下降沿采样。
- **Mode 2**：`CPOL`=1，`CPHA`=0，时钟空闲状态为高电平，数据在下降沿采样。
- **Mode 3**：`CPOL`=1，`CPHA`=1，时钟空闲状态为高电平，数据在上升沿采样。

## BMI088 介绍

BMI088是一款高性能6轴惯性测量单元，集成了一个3轴加速度计和一个3轴陀螺仪，可以测量三维方向上的线加速度以及旋转的角速度。此外，还有一个时钟和一个温度计。

### CUBEMX配置

![img](https://pic1.zhimg.com/v2-5ea1bff269ef4d349d567478725f9e5e_r.jpg)

如图所示配置即可，注意带宽不能超过10MBps。`CPOL`和`CPHA`也可以一起配置为`Low`和`1 edge`。

然后需要配置片选的引脚，因为BMI088的加速度计和陀螺仪需要通过片选选择读取。

在C板上，将`PA4`配置为加速度计的片选引脚，`PB0`配置为陀螺仪的片选引脚，并配置默认为高电平，因为BMI088的片选口是低电平有效。

### 数据读写

#### 端口定义

```c
#define BMI088_SPI hspi1
#define BMI088_ACC_GPIOx GPIOA
#define BMI088_ACC_GPIOp GPIO_PIN_4
#define BMI088_GYRO_GPIOx GPIOB
#define BMI088_GYRO_GPIOp GPIO_PIN_0
```

#### 数据结构体定义

```c
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
```

#### 寄存器表

```c
// bmi088_reg.h
#ifndef __BMI088_REG_H
#define __BMI088_REG_H

#define BMI088_ACCEL_3G_SEN 0.0008974358974f
#define DEG2SEC             0.0174532925f
#define SEC2DEG             57.295779578f
#define PI                  3.14159265f

/*-----加速度计寄存器表-----*/
#define ACC_CHIP_ID_ADDR     0x00
#define ACC_CHIP_ID_VAL      0x1E

#define ACC_ERR_REG_ADDR     0x02

#define ACC_STATUS_ADDR      0x03

#define ACC_X_LSB_ADDR       0x12
#define ACC_X_MSB_ADDR       0x13
#define ACC_Y_LSB_ADDR       0x14
#define ACC_Y_MSB_ADDR       0x15
#define ACC_Z_LSB_ADDR       0x16
#define ACC_Z_MSB_ADDR       0x17
#define ACC_XYZ_LEN          6

#define SENSORTIME_0_ADDR    0x18
#define SENSORTIME_0_UNIT    (39.0625f / 1000000.0f)
#define SENSORTIME_1_ADDR    0x19
#define SENSORTIME_1_UNIT    (10.0 / 1000.0f)
#define SENSORTIME_2_ADDR    0x1A
#define SENSORTIME_2_UNIT    (2.56f)
#define SENSORTIME_LEN       3

#define ACC_INT_STAT_1_ADDR  0x1D

#define TEMP_MSB_ADDR        0x22
#define TEMP_LSB_ADDR        0x23
#define TEMP_LEN             2
#define TEMP_UNIT            0.125f
#define TEMP_BIAS            23.0f

#define ACC_CONF_ADDR        0x40
#define ACC_CONF_RESERVED    0x01
#define ACC_CONF_BWP_OSR4    0x00
#define ACC_CONF_BWP_OSR2    0x01
#define ACC_CONF_BWP_NORM    0x02
#define ACC_CONF_ODR_12_5_Hz 0x05
#define ACC_CONF_ODR_25_Hz   0x06
#define ACC_CONF_ODR_50_Hz   0x07
#define ACC_CONF_ODR_100_Hz  0x08
#define ACC_CONF_ODR_200_Hz  0x09
#define ACC_CONF_ODR_400_Hz  0x0A
#define ACC_CONF_ODR_800_Hz  0x0B
#define ACC_CONF_ODR_1600_Hz 0x0C

#define ACC_RANGE_ADDR       0x41
#define ACC_RANGE_3G         0x00
#define ACC_RANGE_6G         0x01
#define ACC_RANGE_12G        0x02
#define ACC_RANGE_24G        0x03

#define INT1_IO_CTRL_ADDR    0x53

#define INT2_IO_CTRL_ADDR    0x54

#define INT_MAP_DATA_ADDR    0x58

#define ACC_SELF_TEST_ADDR   0x6D
#define ACC_SELF_TEST_OFF    0x00
#define ACC_SELF_TEST_POS    0x0D
#define ACC_SELF_TEST_NEG    0x09

#define ACC_PWR_CONF_ADDR    0x7C
#define ACC_PWR_CONF_SUS     0x03
#define ACC_PWR_CONF_ACT     0x00

#define ACC_PWR_CTRL_ADDR    0x7D
#define ACC_PWR_CTRL_ON      0x04
#define ACC_PWR_CTRL_OFF     0x00

#define ACC_SOFTRESET_ADDR   0x7E
#define ACC_SOFTRESET_VAL    0xB6

/*-----陀螺仪寄存器表-----*/
#define GYRO_CHIP_ID_ADDR               0x00
#define GYRO_CHIP_ID_VAL                0x0F

#define GYRO_RATE_X_LSB_ADDR            0x02
#define GYRO_RATE_X_MSB_ADDR            0x03
#define GYRO_RATE_Y_LSB_ADDR            0x04
#define GYRO_RATE_Y_MSB_ADDR            0x05
#define GYRO_RATE_Z_LSB_ADDR            0x06
#define GYRO_RATE_Z_MSB_ADDR            0x07
#define GYRO_XYZ_LEN                    6

#define GYRO_INT_STAT_1_ADDR            0x0A

#define GYRO_RANGE_ADDR                 0x0F
#define GYRO_RANGE_2000_DEG_S           0x00
#define GYRO_RANGE_1000_DEG_S           0x01
#define GYRO_RANGE_500_DEG_S            0x02
#define GYRO_RANGE_250_DEG_S            0x03
#define GYRO_RANGE_125_DEG_S            0x04

#define GYRO_BANDWIDTH_ADDR             0x10
#define GYRO_ODR_2000Hz_BANDWIDTH_532Hz 0x00
#define GYRO_ODR_2000Hz_BANDWIDTH_230Hz 0x01
#define GYRO_ODR_1000Hz_BANDWIDTH_116Hz 0x02
#define GYRO_ODR_400Hz_BANDWIDTH_47Hz   0x03
#define GYRO_ODR_200Hz_BANDWIDTH_23Hz   0x04
#define GYRO_ODR_100Hz_BANDWIDTH_12Hz   0x05
#define GYRO_ODR_200Hz_BANDWIDTH_64Hz   0x06
#define GYRO_ODR_100Hz_BANDWIDTH_32Hz   0x07

#define GYRO_LPM1_ADDR                  0x11
#define GYRO_LPM1_NOR                   0x00
#define GYRO_LPM1_SUS                   0x80
#define GYRO_LPM1_DEEP_SUS              0x20

#define GYRO_SOFTRESET_ADDR             0x14
#define GYRO_SOFTRESET_VAL              0xB6

#define GYRO_INT_CTRL_ADDR              0x15

#define GYRO_INT3_INT4_IO_CONF_ADDR     0x16

#define GYRO_INT3_INT4_IO_MAP_ADDR      0x18

#define GYRO_SELF_TEST_ADDR             0x3C
#define GYRO_SELF_TEST_ON               0x01

#endif // !__BMI088_REG_H
```

#### 基本读写流程

先向BMI088发送地址，地址为7位，发送的1个字节的剩余一位，依旧是最高位，为0时表示要写数据，为1时表示要读数据。如果要写数据，则继续发送数据即可；如果要读数据，根据官方手册，读到的第一个字节不准确，需要舍弃然后再读一次。

先定义读写标志位的宏定义

```c
#define BMI088_SPI_WRITE_CODE 0x7F
#define BMI088_SPI_READ_CODE  0x80
```

然后则是读写函数的实现。

```c
void WriteDataToGyro(uint8_t addr, uint8_t data)
{
    HAL_GPIO_WritePin(BMI088_GYRO_GPIOx, BMI088_GYRO_GPIOp, GPIO_PIN_RESET);

    // 发送地址
    uint8_t pTxData = (addr & BMI088_SPI_WRITE_CODE);
    HAL_SPI_Transmit(&BMI088_SPI, &pTxData, 1, 1000);
    while (HAL_SPI_GetState(&BMI088_SPI) == HAL_SPI_STATE_BUSY_TX)
        ;

    // 发送数据
    pTxData = data;
    HAL_SPI_Transmit(&BMI088_SPI, &pTxData, 1, 1000);
    while (HAL_SPI_GetState(&BMI088_SPI) == HAL_SPI_STATE_BUSY_TX)
        ;

    HAL_Delay(1);
    HAL_GPIO_WritePin(BMI088_GYRO_GPIOx, BMI088_GYRO_GPIOp, GPIO_PIN_SET);
}

void ReadSingleDataFromGyro(uint8_t addr, uint8_t *data)
{
    HAL_GPIO_WritePin(BMI088_GYRO_GPIOx, BMI088_GYRO_GPIOp, GPIO_PIN_RESET);

    // 发送地址
    uint8_t pTxData = (addr | BMI088_SPI_READ_CODE);
    HAL_SPI_Transmit(&BMI088_SPI, &pTxData, 1, 1000);
    while (HAL_SPI_GetState(&BMI088_SPI) == HAL_SPI_STATE_BUSY_TX)
        ;

    // 接受数据
    HAL_SPI_Receive(&BMI088_SPI, data, 1, 1000);
    while (HAL_SPI_GetState(&BMI088_SPI) == HAL_SPI_STATE_BUSY_RX)
        ;
    HAL_GPIO_WritePin(BMI088_GYRO_GPIOx, BMI088_GYRO_GPIOp, GPIO_PIN_SET);
}

void ReadMultiDataFromGyro(uint8_t addr, uint8_t len, uint8_t *data)
{
    HAL_GPIO_WritePin(BMI088_GYRO_GPIOx, BMI088_GYRO_GPIOp, GPIO_PIN_RESET);

    // 发送地址
    uint8_t pTxData = (addr | BMI088_SPI_READ_CODE);
    uint8_t pRxData;
    HAL_SPI_Transmit(&BMI088_SPI, &pTxData, 1, 1000);
    while (HAL_SPI_GetState(&BMI088_SPI) == HAL_SPI_STATE_BUSY_TX)
        ;

    // 连续接受多个数据
    for (int i = 0; i < len; i++)
    {
        HAL_SPI_Receive(&BMI088_SPI, &pRxData, 1, 1000);
        while (HAL_SPI_GetState(&BMI088_SPI) == HAL_SPI_STATE_BUSY_RX)
            ;
        data[i] = pRxData;
    }
    HAL_GPIO_WritePin(BMI088_GYRO_GPIOx, BMI088_GYRO_GPIOp, GPIO_PIN_SET);
}

void WriteDataToAcc(uint8_t addr, uint8_t data)
{
    HAL_GPIO_WritePin(BMI088_ACC_GPIOx, BMI088_ACC_GPIOp, GPIO_PIN_RESET);

    // 发送地址
    uint8_t pTxData = (addr & BMI088_SPI_WRITE_CODE);
    HAL_SPI_Transmit(&BMI088_SPI, &pTxData, 1, 1000);
    while (HAL_SPI_GetState(&BMI088_SPI) == HAL_SPI_STATE_BUSY_TX)
        ;

    // 发送数据
    pTxData = data;
    HAL_SPI_Transmit(&BMI088_SPI, &pTxData, 1, 1000);
    while (HAL_SPI_GetState(&BMI088_SPI) == HAL_SPI_STATE_BUSY_TX)
        ;

    HAL_Delay(1);
    HAL_GPIO_WritePin(BMI088_ACC_GPIOx, BMI088_ACC_GPIOp, GPIO_PIN_SET);
}

void ReadSingleDataFromAcc(uint8_t addr, uint8_t *data)
{
    HAL_GPIO_WritePin(BMI088_ACC_GPIOx, BMI088_ACC_GPIOp, GPIO_PIN_RESET);

    // 发送地址
    uint8_t pTxData = (addr | BMI088_SPI_READ_CODE);
    HAL_SPI_Transmit(&BMI088_SPI, &pTxData, 1, 1000);
    while (HAL_SPI_GetState(&BMI088_SPI) == HAL_SPI_STATE_BUSY_TX)
        ;

    // 接受数据
    HAL_SPI_Receive(&BMI088_SPI, data, 1, 1000);
    while (HAL_SPI_GetState(&BMI088_SPI) == HAL_SPI_STATE_BUSY_RX)
        ;

    // 再次接受数据，以此避免初次读取到混乱数据
    HAL_SPI_Receive(&BMI088_SPI, data, 1, 1000);
    while (HAL_SPI_GetState(&BMI088_SPI) == HAL_SPI_STATE_BUSY_RX)
        ;

    HAL_GPIO_WritePin(BMI088_ACC_GPIOx, BMI088_ACC_GPIOp, GPIO_PIN_SET);
}

void ReadMultiDataFromAcc(uint8_t addr, uint8_t len, uint8_t *data)
{
    HAL_GPIO_WritePin(BMI088_ACC_GPIOx, BMI088_ACC_GPIOp, GPIO_PIN_RESET);

    // 发送地址
    uint8_t pTxData = (addr | BMI088_SPI_READ_CODE);
    uint8_t pRxData;
    HAL_SPI_Transmit(&BMI088_SPI, &pTxData, 1, 1000);
    while (HAL_SPI_GetState(&BMI088_SPI) == HAL_SPI_STATE_BUSY_TX)
        ;

    // 接受第一个字节的混乱数据
    HAL_SPI_Receive(&BMI088_SPI, &pRxData, 1, 1000);
    while (HAL_SPI_GetState(&BMI088_SPI) == HAL_SPI_STATE_BUSY_RX)
        ;

    // 接受之后的正常数据
    for (int i = 0; i < len; i++)
    {
        HAL_SPI_Receive(&BMI088_SPI, &pRxData, 1, 1000);
        while (HAL_SPI_GetState(&BMI088_SPI) == HAL_SPI_STATE_BUSY_RX)
            ;
        data[i] = pRxData;
    }

    HAL_GPIO_WritePin(BMI088_ACC_GPIOx, BMI088_ACC_GPIOp, GPIO_PIN_SET);
}
```

#### 读取有意义的数据

这部分就是对上面的基础函数的应用。

```c
void ReadAccData(Acc_Raw_Data_Typedef *data)
{
    uint8_t buf[ACC_XYZ_LEN], range;
    int16_t acc[3];
    ReadSingleDataFromAcc(ACC_RANGE_ADDR, &range);
    ReadMultiDataFromAcc(ACC_X_LSB_ADDR, ACC_XYZ_LEN, buf);
    acc[0] = ((int16_t)buf[1] << 8) + (int16_t)buf[0];
    acc[1] = ((int16_t)buf[3] << 8) + (int16_t)buf[2];
    acc[2] = ((int16_t)buf[5] << 8) + (int16_t)buf[4];
    data->x = (float)acc[0] * BMI088_ACCEL_3G_SEN;
    data->y = (float)acc[1] * BMI088_ACCEL_3G_SEN;
    data->z = (float)acc[2] * BMI088_ACCEL_3G_SEN;
}

void ReadGyroData(Gyro_Raw_Data_Typedef *data)
{
    uint8_t buf[GYRO_XYZ_LEN], range;
    int16_t gyro[3];
    float unit;
    ReadSingleDataFromGyro(GYRO_RANGE_ADDR, &range);
    switch (range)
    {
    case 0x00:
        unit = 16.384;
        break;
    case 0x01:
        unit = 32.768;
        break;
    case 0x02:
        unit = 65.536;
        break;
    case 0x03:
        unit = 131.072;
        break;
    case 0x04:
        unit = 262.144;
        break;
    default:
        unit = 16.384;
        break;
    }
    ReadMultiDataFromGyro(GYRO_RATE_X_LSB_ADDR, GYRO_XYZ_LEN, buf);
    gyro[0] = ((int16_t)buf[1] << 8) + (int16_t)buf[0];
    gyro[1] = ((int16_t)buf[3] << 8) + (int16_t)buf[2];
    gyro[2] = ((int16_t)buf[5] << 8) + (int16_t)buf[4];
    data->roll = (float)gyro[0] / unit * DEG2SEC;
    data->pitch = (float)gyro[1] / unit * DEG2SEC;
    data->yaw = (float)gyro[2] / unit * DEG2SEC;
}

void ReadAccSensorTime(float *time)
{
    uint8_t buf[SENSORTIME_LEN];
    ReadMultiDataFromAcc(SENSORTIME_0_ADDR, SENSORTIME_LEN, buf);
    *time = buf[0] * SENSORTIME_0_UNIT + buf[1] * SENSORTIME_1_UNIT + buf[2] * SENSORTIME_2_UNIT;
}

void ReadAccTemperature(float *temp)
{
    uint8_t buf[TEMP_LEN];
    ReadMultiDataFromAcc(TEMP_MSB_ADDR, TEMP_LEN, buf);
    uint16_t temp_uint11 = (buf[0] << 3) + (buf[1] >> 5);
    int16_t temp_int11;
    if (temp_uint11 > 1023)
    {
        temp_int11 = (int16_t)temp_uint11 - 2048;
    }
    else
    {
        temp_int11 = (int16_t)temp_uint11;
    }
    *temp = temp_int11 * TEMP_UNIT + TEMP_BIAS;
}
```

#### 初始化函数

```c
bool BMI088_Init_Flag = false;

BMI088_Error BMI088_Init()
{
    BMI088_Error error = NO_ERR;

    BMI088_Conf_Init();

    error |= VerifyAccChipID();
    error |= VerifyGyroChipID();
    if (1)
    { // 将来改成变量控制自检
        error |= VerifyAccSelfTest();
    }
    if (1)
    { // 将来改成变量控制自检
        error |= VerifyGyroSelfTest();
    }
    BMI088_Init_Flag = true;
    return error;
}

void BMI088_Conf_Init()
{
    // 加速度计初始化
    // 先软重启，清空所有寄存器
    WriteDataToAcc(ACC_SOFTRESET_ADDR, ACC_SOFTRESET_VAL);
    HAL_Delay(50);
    // 打开加速度计电源
    WriteDataToAcc(ACC_PWR_CTRL_ADDR, ACC_PWR_CTRL_ON);
    // 加速度计变成正常模式
    WriteDataToAcc(ACC_PWR_CONF_ADDR, ACC_PWR_CONF_ACT);

    // 陀螺仪初始化
    // 先软重启，清空所有寄存器
    WriteDataToGyro(GYRO_SOFTRESET_ADDR, GYRO_SOFTRESET_VAL);
    HAL_Delay(50);
    // 陀螺仪变成正常模式
    WriteDataToGyro(GYRO_LPM1_ADDR, GYRO_LPM1_NOR);

    // 加速度计配置写入
    // 写入范围，+-3g的测量范围
    WriteDataToAcc(ACC_RANGE_ADDR, ACC_RANGE_3G);
    // 写入配置，正常带宽，1600hz输出频率
    WriteDataToAcc(ACC_CONF_ADDR,
                   (ACC_CONF_RESERVED << 7) | (ACC_CONF_BWP_NORM << 6) | (ACC_CONF_ODR_1600_Hz));

    // 陀螺仪配置写入
    // 写入范围，+-500°/s的测量范围
    WriteDataToGyro(GYRO_RANGE_ADDR, GYRO_RANGE_500_DEG_S);
    // 写入带宽，2000Hz输出频率，532Hz滤波器带宽
    WriteDataToGyro(GYRO_BANDWIDTH_ADDR, GYRO_ODR_2000Hz_BANDWIDTH_532Hz);
}

BMI088_Error VerifyAccChipID(void)
{
    uint8_t chip_id;
    ReadSingleDataFromAcc(ACC_CHIP_ID_ADDR, &chip_id);
    if (chip_id != ACC_CHIP_ID_VAL)
    {
        return ACC_CHIP_ID_ERR;
    }
    return NO_ERR;
}

BMI088_Error VerifyGyroChipID(void)
{
    uint8_t chip_id;
    ReadSingleDataFromGyro(GYRO_CHIP_ID_ADDR, &chip_id);
    if (chip_id != GYRO_CHIP_ID_VAL)
    {
        return GYRO_CHIP_ID_ERR;
    }
    return NO_ERR;
}

BMI088_Error VerifyAccSelfTest(void)
{
    Acc_Raw_Data_Typedef pos_data, neg_data;
    WriteDataToAcc(ACC_RANGE_ADDR, ACC_RANGE_24G);
    WriteDataToAcc(ACC_CONF_ADDR, 0xA7);
    HAL_Delay(10);
    WriteDataToAcc(ACC_SELF_TEST_ADDR, ACC_SELF_TEST_POS);
    HAL_Delay(100);
    ReadAccData(&pos_data);
    WriteDataToAcc(ACC_SELF_TEST_ADDR, ACC_SELF_TEST_NEG);
    HAL_Delay(100);
    ReadAccData(&neg_data);
    WriteDataToAcc(ACC_SELF_TEST_ADDR, ACC_SELF_TEST_OFF);
    HAL_Delay(100);
    if ((fabs(pos_data.x - neg_data.x) > 0.1f) || (fabs(pos_data.y - neg_data.y) > 0.1f) || (fabs(pos_data.z - neg_data.z) > 0.1f))
    {
        return ACC_DATA_ERR;
    }
    WriteDataToAcc(ACC_SOFTRESET_ADDR, ACC_SOFTRESET_VAL);
    WriteDataToAcc(ACC_PWR_CTRL_ADDR, ACC_PWR_CTRL_ON);
    WriteDataToAcc(ACC_PWR_CONF_ADDR, ACC_PWR_CONF_ACT);
    WriteDataToAcc(ACC_CONF_ADDR,
                   (ACC_CONF_RESERVED << 7) | (ACC_CONF_BWP_NORM << 6) | (ACC_CONF_ODR_1600_Hz));
    WriteDataToAcc(ACC_RANGE_ADDR, ACC_RANGE_3G);
    return NO_ERR;
}

BMI088_Error VerifyGyroSelfTest(void)
{
    WriteDataToGyro(GYRO_SELF_TEST_ADDR, GYRO_SELF_TEST_ON);
    uint8_t bist_rdy = 0x00, bist_fail;
    while (bist_rdy == 0)
    {
        ReadSingleDataFromGyro(GYRO_SELF_TEST_ADDR, &bist_rdy);
        bist_rdy = (bist_rdy & 0x02) >> 1;
    }
    ReadSingleDataFromGyro(GYRO_SELF_TEST_ADDR, &bist_fail);
    bist_fail = (bist_fail & 0x04) >> 2;
    if (bist_fail == 0)
    {
        return NO_ERR;
    }
    else
    {
        return GYRO_DATA_ERR;
    }
}
```

### 数据滤波

此部分为纯数学部分，不详细阐述，仅仅简单列举。具体的算法实现可交给AI，数学原理并非本文的重点。常见的滤波方法如下：

1. 移动平均滤波
2. 一阶低通滤波
3. 互补滤波
4. `Mahony`滤波
5. 卡尔板滤波 / 拓展卡尔曼滤波

其中，前两者只是单纯对原始数据进行滤波，后三者在滤波的同时，还能得到对姿态的解算。

## 解算姿态

### 描述姿态

#### 欧拉角

通过`roll`、`pitch`、`yaw`三个方向的旋转角度描述姿态，但有万向锁问题，会导致`pitch`的角度只有-90~90度，会丢失信息。

![三维旋转：欧拉角、四元数、旋转矩阵、轴角之间的转换 - 知乎](https://pic3.zhimg.com/v2-9e1b5ce7917863ea39d34e84f3884faa_r.jpg)

#### 四元数

四元数可视为一个拓展的复数，可以解决万向锁问题。具体的原理讲解详见[3B1B的视频](https://www.bilibili.com/video/av33385105/)



![数学：四元数 - lnlidawei - 博客园](https://img2022.cnblogs.com/blog/1524989/202211/1524989-20221125161940645-1222989634.png)

### 计算姿态

这部分主要采用的方法就是上面数据滤波种所介绍的后三种方法。

#### 代码

这部分代码主要是数学计算，比较长且非重点，具体详见`filters.h/c`和`attitude_ekf.h/c`文件

## 测试用代码编写

导入`attitude_ekf.h`头文件

然后在无限循环前进行初始化

```c
BMI088_Init();
BMI088_Attitude_Init(0.01f);
```

在无限循环中，每10ms进行一次解算。

```c
BMI088_Attitude_Update();
const AttitudeData *attitude = BMI088_GetAttitudeData();
HAL_Delay(10);
```

具体效果见目录下的`Videos`文件夹

## 总结

本周学习了SPI和BMI088的使用，学会了如何使用BMI088进行运动姿态的解算。在解算过程中，需要用到一些数学知识和算法，这是我目前较为薄弱和需要提升的地方。毕竟只有完全理解了算法才能进一步优化和改进，并且算法在之后更多的地方都会发挥很大的作用，需要重视。

