// bmi088_attitude.c
#include "bmi088.h"
#include "filters.h"
#include "attitude_ekf.h"
#include <math.h>
#include "string.h"

// 滤波器实例
static LowPassFilter acc_filter_x, acc_filter_y, acc_filter_z;
static LowPassFilter gyro_filter_x, gyro_filter_y, gyro_filter_z;

// 滑动平均滤波器缓冲区
static float acc_buffer_x[5], acc_buffer_y[5], acc_buffer_z[5];
static float gyro_buffer_x[5], gyro_buffer_y[5], gyro_buffer_z[5];
static MovingAverageFilter acc_maf_x, acc_maf_y, acc_maf_z;
static MovingAverageFilter gyro_maf_x, gyro_maf_y, gyro_maf_z;

// EKF实例
static AttitudeEKF attitude_ekf;

// 姿态数据结构
typedef struct {
    EulerAngles angles_deg; // 欧拉角（度）
    EulerAngles angles_rad; // 欧拉角（弧度）
    Quaternion quaternion;  // 四元数
    float temperature;      // 温度
    uint32_t timestamp_ms;  // 时间戳
} AttitudeData;

AttitudeData current_attitude;

// 初始化滤波器和EKF
void BMI088_Attitude_Init(float dt)
{
    // 初始化低通滤波器 (alpha = 0.2)
    LowPassFilter_Init(&acc_filter_x, 0.2f, 0.0f);
    LowPassFilter_Init(&acc_filter_y, 0.2f, 0.0f);
    LowPassFilter_Init(&acc_filter_z, 0.2f, 0.0f);

    LowPassFilter_Init(&gyro_filter_x, 0.3f, 0.0f);
    LowPassFilter_Init(&gyro_filter_y, 0.3f, 0.0f);
    LowPassFilter_Init(&gyro_filter_z, 0.3f, 0.0f);

    // 初始化滑动平均滤波器
    MovingAverageFilter_Init(&acc_maf_x, acc_buffer_x, 5);
    MovingAverageFilter_Init(&acc_maf_y, acc_buffer_y, 5);
    MovingAverageFilter_Init(&acc_maf_z, acc_buffer_z, 5);

    MovingAverageFilter_Init(&gyro_maf_x, gyro_buffer_x, 5);
    MovingAverageFilter_Init(&gyro_maf_y, gyro_buffer_y, 5);
    MovingAverageFilter_Init(&gyro_maf_z, gyro_buffer_z, 5);

    // 初始化EKF
    AttitudeEKF_Init(&attitude_ekf, dt);

    // 初始化姿态数据
    memset(&current_attitude, 0, sizeof(current_attitude));
}

// 处理原始数据并进行姿态解算
void BMI088_Attitude_Update(void)
{
    static uint32_t last_time_ms = 0;
    uint32_t current_time_ms     = HAL_GetTick();
    float dt                     = (current_time_ms - last_time_ms) / 1000.0f;

    if (dt <= 0) dt = 0.001f;  // 默认1ms
    if (dt > 0.1f) dt = 0.01f; // 限制最大时间间隔

    // 读取原始数据
    Acc_Raw_Data_Typedef acc_raw;
    Gyro_Raw_Data_Typedef gyro_raw;
    ReadAccData(&acc_raw);
    ReadGyroData(&gyro_raw);

    // 从rad/s转换到deg/s（如果需要）
    float gyro_x_deg = gyro_raw.roll;  // 注意：roll对应绕X轴的旋转
    float gyro_y_deg = gyro_raw.pitch; // pitch对应绕Y轴的旋转
    float gyro_z_deg = gyro_raw.yaw;   // yaw对应绕Z轴的旋转

    // 应用滤波处理
    // 1. 滑动平均滤波
    float acc_x_filtered = MovingAverageFilter_Process(&acc_maf_x, acc_raw.x);
    float acc_y_filtered = MovingAverageFilter_Process(&acc_maf_y, acc_raw.y);
    float acc_z_filtered = MovingAverageFilter_Process(&acc_maf_z, acc_raw.z);

    float gyro_x_filtered = MovingAverageFilter_Process(&gyro_maf_x, gyro_x_deg);
    float gyro_y_filtered = MovingAverageFilter_Process(&gyro_maf_y, gyro_y_deg);
    float gyro_z_filtered = MovingAverageFilter_Process(&gyro_maf_z, gyro_z_deg);

    // 2. 低通滤波
    acc_x_filtered = LowPassFilter_Process(&acc_filter_x, acc_x_filtered);
    acc_y_filtered = LowPassFilter_Process(&acc_filter_y, acc_y_filtered);
    acc_z_filtered = LowPassFilter_Process(&acc_filter_z, acc_z_filtered);

    gyro_x_filtered = LowPassFilter_Process(&gyro_filter_x, gyro_x_filtered);
    gyro_y_filtered = LowPassFilter_Process(&gyro_filter_y, gyro_y_filtered);
    gyro_z_filtered = LowPassFilter_Process(&gyro_filter_z, gyro_z_filtered);

    // 检查加速度数据有效性（排除剧烈运动）
    float acc_magnitude = sqrtf(acc_x_filtered * acc_x_filtered +
                                acc_y_filtered * acc_y_filtered +
                                acc_z_filtered * acc_z_filtered);

    // 加速度量级应在0.8g-1.2g范围内才用于更新
    const float g      = 9.80665f;
    int use_acc_update = (acc_magnitude > 0.8f * g && acc_magnitude < 1.2f * g);

    // 将deg/s转换为rad/s用于EKF
    float gyro_x_rad = gyro_x_filtered;
    float gyro_y_rad = gyro_y_filtered;
    float gyro_z_rad = gyro_z_filtered;

    // EKF预测步骤（使用陀螺仪）
    AttitudeEKF_Predict(&attitude_ekf, gyro_x_rad, gyro_y_rad, gyro_z_rad);

    // EKF更新步骤（使用加速度计，仅在量级合适时）
    if (use_acc_update) {
        AttitudeEKF_UpdateWithAcc(&attitude_ekf, acc_x_filtered, acc_y_filtered, acc_z_filtered);
    }

    // 获取解算结果
    AttitudeEKF_GetQuaternion(&attitude_ekf, &current_attitude.quaternion);
    AttitudeEKF_GetEulerAnglesDeg(&attitude_ekf, &current_attitude.angles_deg);
    AttitudeEKF_GetEulerAngles(&attitude_ekf, &current_attitude.angles_rad);

    // 读取温度
    ReadAccTemperature(&current_attitude.temperature);

    // 更新时间戳
    current_attitude.timestamp_ms = current_time_ms;
    last_time_ms                  = current_time_ms;
}

// 获取当前姿态数据
const AttitudeData *BMI088_GetAttitudeData(void)
{
    return &current_attitude;
}
