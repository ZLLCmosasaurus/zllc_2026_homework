// attitude_ekf.c
#include "attitude_ekf.h"
#include <math.h>
#include <string.h>

#define PI 3.14159265358979323846f

// 重力加速度大小 (m/s^2)
#define GRAVITY 9.80665f

// 向量点积
static float vector_dot(const float *a, const float *b, int n) {
    float sum = 0.0f;
    for (int i = 0; i < n; i++) {
        sum += a[i] * b[i];
    }
    return sum;
}

// 矩阵乘法: C = A * B
static void matrix_multiply(const float A[7][7], const float B[7][7], float C[7][7]) {
    for (int i = 0; i < 7; i++) {
        for (int j = 0; j < 7; j++) {
            C[i][j] = 0.0f;
            for (int k = 0; k < 7; k++) {
                C[i][j] += A[i][k] * B[k][j];
            }
        }
    }
}

// 矩阵转置
static void matrix_transpose(const float A[7][7], float AT[7][7]) {
    for (int i = 0; i < 7; i++) {
        for (int j = 0; j < 7; j++) {
            AT[j][i] = A[i][j];
        }
    }
}

void AttitudeEKF_Init(AttitudeEKF *ekf, float dt) {
    memset(ekf, 0, sizeof(AttitudeEKF));
    
    // 初始化状态向量（单位四元数）
    ekf->x[0] = 1.0f;  // q0
    ekf->x[1] = 0.0f;  // q1
    ekf->x[2] = 0.0f;  // q2
    ekf->x[3] = 0.0f;  // q3
    ekf->x[4] = 0.0f;  // wx_bias
    ekf->x[5] = 0.0f;  // wy_bias
    ekf->x[6] = 0.0f;  // wz_bias
    
    // 初始化协方差矩阵
    for (int i = 0; i < EKF_STATE_DIM; i++) {
        for (int j = 0; j < EKF_STATE_DIM; j++) {
            ekf->P[i][j] = (i == j) ? 1.0f : 0.0f;
        }
    }
    
    // 初始化过程噪声协方差矩阵
    for (int i = 0; i < EKF_STATE_DIM; i++) {
        for (int j = 0; j < EKF_STATE_DIM; j++) {
            if (i == j) {
                if (i < 4) {
                    ekf->Q[i][j] = 1e-6f;    // 四元数过程噪声
                } else {
                    ekf->Q[i][j] = 1e-8f;    // 陀螺零偏过程噪声
                }
            } else {
                ekf->Q[i][j] = 0.0f;
            }
        }
    }
    
    // 初始化观测噪声协方差矩阵
    ekf->R_acc[0] = 0.1f;
    ekf->R_acc[1] = 0.1f;
    ekf->R_acc[2] = 0.1f;
    
    ekf->dt = dt;
    ekf->initialized = true;
    
    // 默认地磁场参考向量（北京地区，单位：uT）
    ekf->mag_ref[0] = 30.0f;   // North
    ekf->mag_ref[1] = 0.0f;    // East
    ekf->mag_ref[2] = -50.0f;  // Down
}

void AttitudeEKF_SetMagReference(AttitudeEKF *ekf, float mx, float my, float mz) {
    ekf->mag_ref[0] = mx;
    ekf->mag_ref[1] = my;
    ekf->mag_ref[2] = mz;
}

void AttitudeEKF_Predict(AttitudeEKF *ekf, float wx, float wy, float wz) {
    if (!ekf->initialized) return;
    
    // 从状态向量中提取四元数和零偏
    Quaternion q = {ekf->x[0], ekf->x[1], ekf->x[2], ekf->x[3]};
    float wx_bias = ekf->x[4];
    float wy_bias = ekf->x[5];
    float wz_bias = ekf->x[6];
    
    // 去除零偏的角速度
    float wx_corrected = wx - wx_bias;
    float wy_corrected = wy - wy_bias;
    float wz_corrected = wz - wz_bias;
    
    // 四元数更新（使用一阶龙格库塔法）
    float norm = sqrtf(wx_corrected * wx_corrected + 
                       wy_corrected * wy_corrected + 
                       wz_corrected * wz_corrected);
    
    if (norm > 1e-6f) {
        float angle = norm * ekf->dt * 0.5f;
        float sin_angle = sinf(angle);
        float cos_angle = cosf(angle);
        
        float k0 = cos_angle * q.q0 - sin_angle * (wx_corrected * q.q1 + 
                                                   wy_corrected * q.q2 + 
                                                   wz_corrected * q.q3) / norm;
        float k1 = cos_angle * q.q1 + sin_angle * (wx_corrected * q.q0 + 
                                                   wz_corrected * q.q2 - 
                                                   wy_corrected * q.q3) / norm;
        float k2 = cos_angle * q.q2 + sin_angle * (wy_corrected * q.q0 - 
                                                   wz_corrected * q.q1 + 
                                                   wx_corrected * q.q3) / norm;
        float k3 = cos_angle * q.q3 + sin_angle * (wz_corrected * q.q0 + 
                                                   wy_corrected * q.q1 - 
                                                   wx_corrected * q.q2) / norm;
        
        // 归一化
        float norm_q = sqrtf(k0*k0 + k1*k1 + k2*k2 + k3*k3);
        if (norm_q > 1e-6f) {
            ekf->x[0] = k0 / norm_q;
            ekf->x[1] = k1 / norm_q;
            ekf->x[2] = k2 / norm_q;
            ekf->x[3] = k3 / norm_q;
        }
    }
    
    // 更新协方差矩阵 P = F * P * F^T + Q
    // 简化的处理：P = P + Q
    for (int i = 0; i < EKF_STATE_DIM; i++) {
        for (int j = 0; j < EKF_STATE_DIM; j++) {
            ekf->P[i][j] += ekf->Q[i][j];
        }
    }
}

void AttitudeEKF_UpdateWithAcc(AttitudeEKF *ekf, float ax, float ay, float az) {
    if (!ekf->initialized) return;
    
    // 从状态向量中提取四元数
    float q0 = ekf->x[0];
    float q1 = ekf->x[1];
    float q2 = ekf->x[2];
    float q3 = ekf->x[3];
    
    // 归一化加速度计测量值
    float norm_a = sqrtf(ax*ax + ay*ay + az*az);
    if (norm_a < 1e-6f) return;
    ax /= norm_a;
    ay /= norm_a;
    az /= norm_a;
    
    // 预测的重力向量（在机体坐标系中）
    // g_body = R^T * [0, 0, g]
    float gx_pred = 2.0f * (q1*q3 - q0*q2);
    float gy_pred = 2.0f * (q0*q1 + q2*q3);
    float gz_pred = q0*q0 - q1*q1 - q2*q2 + q3*q3;
    
    // 测量残差
    float dz[3];
    dz[0] = ax - gx_pred;
    dz[1] = ay - gy_pred;
    dz[2] = az - gz_pred;
    
    // 观测矩阵 H
    float H[3][7] = {0};
    
    H[0][0] = -2*q2; H[0][1] = 2*q3; H[0][2] = -2*q0; H[0][3] = 2*q1;
    H[1][0] = 2*q1;  H[1][1] = 2*q0; H[1][2] = 2*q3;  H[1][3] = 2*q2;
    H[2][0] = 2*q0;  H[2][1] = -2*q1; H[2][2] = -2*q2; H[2][3] = 2*q3;
    
    // 计算卡尔曼增益 K = P * H^T * (H * P * H^T + R)^(-1)
    float PH_T[7][3] = {0};
    
    // PH_T = P * H^T
    for (int i = 0; i < 7; i++) {
        for (int j = 0; j < 3; j++) {
            PH_T[i][j] = 0.0f;
            for (int k = 0; k < 7; k++) {
                PH_T[i][j] += ekf->P[i][k] * H[j][k];
            }
        }
    }
    
    // S = H * P * H^T + R
    float S[3][3] = {0};
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            S[i][j] = 0.0f;
            for (int k = 0; k < 7; k++) {
                S[i][j] += H[i][k] * PH_T[k][j];
            }
            if (i == j) {
                S[i][j] += ekf->R_acc[i];
            }
        }
    }
    
    // 计算 S 的逆（3x3矩阵求逆）
    float det_S = S[0][0] * (S[1][1]*S[2][2] - S[2][1]*S[1][2]) -
                  S[0][1] * (S[1][0]*S[2][2] - S[1][2]*S[2][0]) +
                  S[0][2] * (S[1][0]*S[2][1] - S[1][1]*S[2][0]);
    
    if (fabsf(det_S) < 1e-6f) return;
    
    float inv_S[3][3];
    inv_S[0][0] = (S[1][1]*S[2][2] - S[2][1]*S[1][2]) / det_S;
    inv_S[0][1] = (S[0][2]*S[2][1] - S[0][1]*S[2][2]) / det_S;
    inv_S[0][2] = (S[0][1]*S[1][2] - S[0][2]*S[1][1]) / det_S;
    inv_S[1][0] = (S[1][2]*S[2][0] - S[1][0]*S[2][2]) / det_S;
    inv_S[1][1] = (S[0][0]*S[2][2] - S[0][2]*S[2][0]) / det_S;
    inv_S[1][2] = (S[1][0]*S[0][2] - S[0][0]*S[1][2]) / det_S;
    inv_S[2][0] = (S[1][0]*S[2][1] - S[2][0]*S[1][1]) / det_S;
    inv_S[2][1] = (S[2][0]*S[0][1] - S[0][0]*S[2][1]) / det_S;
    inv_S[2][2] = (S[0][0]*S[1][1] - S[1][0]*S[0][1]) / det_S;
    
    // K = PH_T * inv_S
    float K[7][3] = {0};
    for (int i = 0; i < 7; i++) {
        for (int j = 0; j < 3; j++) {
            K[i][j] = 0.0f;
            for (int k = 0; k < 3; k++) {
                K[i][j] += PH_T[i][k] * inv_S[k][j];
            }
        }
    }
    
    // 更新状态向量 x = x + K * dz
    for (int i = 0; i < 7; i++) {
        float dx = 0.0f;
        for (int j = 0; j < 3; j++) {
            dx += K[i][j] * dz[j];
        }
        ekf->x[i] += dx;
    }
    
    // 归一化四元数
    float norm_q = sqrtf(ekf->x[0]*ekf->x[0] + ekf->x[1]*ekf->x[1] + 
                         ekf->x[2]*ekf->x[2] + ekf->x[3]*ekf->x[3]);
    if (norm_q > 1e-6f) {
        ekf->x[0] /= norm_q;
        ekf->x[1] /= norm_q;
        ekf->x[2] /= norm_q;
        ekf->x[3] /= norm_q;
    }
    
    // 更新协方差矩阵 P = (I - K * H) * P
    float KH[7][7] = {0};
    for (int i = 0; i < 7; i++) {
        for (int j = 0; j < 7; j++) {
            KH[i][j] = 0.0f;
            for (int k = 0; k < 3; k++) {
                KH[i][j] += K[i][k] * H[k][j];
            }
        }
    }
    
    float I_minus_KH[7][7];
    for (int i = 0; i < 7; i++) {
        for (int j = 0; j < 7; j++) {
            I_minus_KH[i][j] = (i == j ? 1.0f : 0.0f) - KH[i][j];
        }
    }
    
    float new_P[7][7];
    matrix_multiply(I_minus_KH, ekf->P, new_P);
    memcpy(ekf->P, new_P, sizeof(new_P));
}

void AttitudeEKF_GetQuaternion(const AttitudeEKF *ekf, Quaternion *q) {
    q->q0 = ekf->x[0];
    q->q1 = ekf->x[1];
    q->q2 = ekf->x[2];
    q->q3 = ekf->x[3];
}

void AttitudeEKF_GetEulerAngles(const AttitudeEKF *ekf, EulerAngles *euler) {
    Quaternion q;
    AttitudeEKF_GetQuaternion(ekf, &q);
    Quaternion_ToEulerAngles(&q, euler);
}

void AttitudeEKF_GetEulerAnglesDeg(const AttitudeEKF *ekf, EulerAngles *euler) {
    AttitudeEKF_GetEulerAngles(ekf, euler);
    euler->roll *= RAD2DEG;
    euler->pitch *= RAD2DEG;
    euler->yaw *= RAD2DEG;
}

void Quaternion_ToEulerAngles(const Quaternion *q, EulerAngles *euler) {
    // 使用 ZYX 旋转顺序（roll-pitch-yaw）
    float q0q0 = q->q0 * q->q0;
    float q0q1 = q->q0 * q->q1;
    float q0q2 = q->q0 * q->q2;
    float q0q3 = q->q0 * q->q3;
    float q1q1 = q->q1 * q->q1;
    float q1q2 = q->q1 * q->q2;
    float q1q3 = q->q1 * q->q3;
    float q2q2 = q->q2 * q->q2;
    float q2q3 = q->q2 * q->q3;
    float q3q3 = q->q3 * q->q3;
    
    // Roll (x-axis)
    float sinr_cosp = 2.0f * (q0q1 + q2q3);
    float cosr_cosp = 1.0f - 2.0f * (q1q1 + q2q2);
    euler->roll = atan2f(sinr_cosp, cosr_cosp);
    
    // Pitch (y-axis)
    float sinp = 2.0f * (q0q2 - q1q3);
    if (fabsf(sinp) >= 1.0f) {
        euler->pitch = copysignf(PI / 2.0f, sinp); // 使用 90 度
    } else {
        euler->pitch = asinf(sinp);
    }
    
    // Yaw (z-axis)
    float siny_cosp = 2.0f * (q0q3 + q1q2);
    float cosy_cosp = 1.0f - 2.0f * (q2q2 + q3q3);
    euler->yaw = atan2f(siny_cosp, cosy_cosp);
}

void EulerAngles_ToQuaternion(const EulerAngles *euler, Quaternion *q) {
    float cy = cosf(euler->yaw * 0.5f);
    float sy = sinf(euler->yaw * 0.5f);
    float cp = cosf(euler->pitch * 0.5f);
    float sp = sinf(euler->pitch * 0.5f);
    float cr = cosf(euler->roll * 0.5f);
    float sr = sinf(euler->roll * 0.5f);
    
    q->q0 = cr * cp * cy + sr * sp * sy;
    q->q1 = sr * cp * cy - cr * sp * sy;
    q->q2 = cr * sp * cy + sr * cp * sy;
    q->q3 = cr * cp * sy - sr * sp * cy;
}

void Quaternion_Multiply(const Quaternion *q1, const Quaternion *q2, Quaternion *result) {
    result->q0 = q1->q0 * q2->q0 - q1->q1 * q2->q1 - q1->q2 * q2->q2 - q1->q3 * q2->q3;
    result->q1 = q1->q0 * q2->q1 + q1->q1 * q2->q0 + q1->q2 * q2->q3 - q1->q3 * q2->q2;
    result->q2 = q1->q0 * q2->q2 - q1->q1 * q2->q3 + q1->q2 * q2->q0 + q1->q3 * q2->q1;
    result->q3 = q1->q0 * q2->q3 + q1->q1 * q2->q2 - q1->q2 * q2->q1 + q1->q3 * q2->q0;
}

void Quaternion_Normalize(Quaternion *q) {
    float norm = sqrtf(q->q0 * q->q0 + q->q1 * q->q1 + 
                       q->q2 * q->q2 + q->q3 * q->q3);
    if (norm > 1e-6f) {
        q->q0 /= norm;
        q->q1 /= norm;
        q->q2 /= norm;
        q->q3 /= norm;
    }
}

void Quaternion_Conjugate(const Quaternion *q, Quaternion *result) {
    result->q0 = q->q0;
    result->q1 = -q->q1;
    result->q2 = -q->q2;
    result->q3 = -q->q3;
}