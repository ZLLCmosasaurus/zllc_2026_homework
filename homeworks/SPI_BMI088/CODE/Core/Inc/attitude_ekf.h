// attitude_ekf.h
#ifndef __ATTITUDE_EKF_H
#define __ATTITUDE_EKF_H

#include <stdint.h>
#include <stdbool.h>

#define DEG2RAD (PI / 180.0f)
#define RAD2DEG (180.0f / PI)

// 四元数结构体
typedef struct {
    float q0;   // 实部
    float q1;   // i
    float q2;   // j
    float q3;   // k
} Quaternion;

// 欧拉角结构体
typedef struct {
    float roll;     // 横滚角 (rad)
    float pitch;    // 俯仰角 (rad)
    float yaw;      // 偏航角 (rad)
} EulerAngles;

// 状态向量 [q0, q1, q2, q3, wx_b, wy_b, wz_b]
#define EKF_STATE_DIM 7

// EKF结构体
typedef struct {
    // 状态向量
    float x[EKF_STATE_DIM];
    
    // 协方差矩阵
    float P[EKF_STATE_DIM][EKF_STATE_DIM];
    
    // 过程噪声协方差矩阵
    float Q[EKF_STATE_DIM][EKF_STATE_DIM];
    
    // 观测噪声协方差矩阵
    float R_acc[3];  // 加速度计观测噪声
    float R_mag[3];  // 磁力计观测噪声（可选）
    
    // 时间相关
    float dt;        // 采样时间
    uint32_t last_time_us;
    
    // 标志位
    bool initialized;
    
    // 地磁场向量（单位：uT，在地理坐标系中）
    float mag_ref[3];
} AttitudeEKF;

// 初始化EKF
void AttitudeEKF_Init(AttitudeEKF *ekf, float dt);

// 设置参考磁向量（可选）
void AttitudeEKF_SetMagReference(AttitudeEKF *ekf, float mx, float my, float mz);

// EKF预测步骤（使用陀螺仪数据）
void AttitudeEKF_Predict(AttitudeEKF *ekf, float wx, float wy, float wz);

// EKF更新步骤（使用加速度计数据）
void AttitudeEKF_UpdateWithAcc(AttitudeEKF *ekf, float ax, float ay, float az);

// EKF更新步骤（使用磁力计数据，可选）
void AttitudeEKF_UpdateWithMag(AttitudeEKF *ekf, float mx, float my, float mz);

// 获取当前四元数
void AttitudeEKF_GetQuaternion(const AttitudeEKF *ekf, Quaternion *q);

// 获取当前欧拉角（rad）
void AttitudeEKF_GetEulerAngles(const AttitudeEKF *ekf, EulerAngles *euler);

// 获取当前欧拉角（度）
void AttitudeEKF_GetEulerAnglesDeg(const AttitudeEKF *ekf, EulerAngles *euler);

// 四元数转换为欧拉角
void Quaternion_ToEulerAngles(const Quaternion *q, EulerAngles *euler);

// 欧拉角转换为四元数
void EulerAngles_ToQuaternion(const EulerAngles *euler, Quaternion *q);

// 四元数乘法
void Quaternion_Multiply(const Quaternion *q1, const Quaternion *q2, Quaternion *result);

// 四元数归一化
void Quaternion_Normalize(Quaternion *q);

// 四元数共轭
void Quaternion_Conjugate(const Quaternion *q, Quaternion *result);

#endif