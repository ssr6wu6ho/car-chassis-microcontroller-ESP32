// ekf_mpu6050.cpp
#include "ekf_mpu6050.h"
#include <math.h>
#include <esp_timer.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

EKF_MPU6050::EKF_MPU6050() 
    : ekf13(nullptr), initialized(false), last_time_us(0), dt(0.01f) {
}

EKF_MPU6050::~EKF_MPU6050() {
    if (ekf13) {
        delete ekf13;
    }
}

void EKF_MPU6050::init() {
    if (!ekf13) {
        ekf13 = new ekf_imu13states();
    }
    
    ekf13->Init();
    
    // 设置初始状态 - 四元数为单位四元数，偏差为0
    ekf13->X(0, 0) = 1.0f;  // q0
    ekf13->X(0, 1) = 0.0f;  // q1
    ekf13->X(0, 2) = 0.0f;  // q2
    ekf13->X(0, 3) = 0.0f;  // q3
    
    // 设置测量噪声协方差
    // 对于MPU6050，典型值为:
    // 加速度计噪声: 0.01-0.1 m/s²
    // 陀螺仪过程噪声: 0.001-0.01 rad/s
    // 这些值可能需要根据你的具体传感器调整
    
    initialized = true;
    last_time_us = (float)esp_timer_get_time() / 1000000.0f;
}

void EKF_MPU6050::reset() {
    if (ekf13) {
        delete ekf13;
    }
    ekf13 = new ekf_imu13states();
    init();
}

EKF_MPU6050::Attitude EKF_MPU6050::update(float accel[3], float gyro[3], float dt_in) {
    Attitude result = {0.0f, 0.0f, 0.0f};
    
    if (!initialized || !ekf13) {
        return result;
    }
    
    // 计算时间间隔（秒）
    float current_time_us = (float)esp_timer_get_time() / 1000000.0f;
    if (dt_in > 0.0f) {
        dt = dt_in;
    } else {
        if (last_time_us > 0.0f) {
            dt = current_time_us - last_time_us;
            // 限制dt在合理范围内
            if (dt < 0.001f) dt = 0.001f;
            if (dt > 0.1f) dt = 0.02f;  // 默认20ms
        }
    }
    last_time_us = current_time_us;
    
    // 归一化加速度计数据（假设输入为m/s²）
    float norm_accel = sqrt(accel[0]*accel[0] + accel[1]*accel[1] + accel[2]*accel[2]);
    float accel_norm[3];
    if (norm_accel > 0.001f) {
        accel_norm[0] = accel[0] / norm_accel;
        accel_norm[1] = accel[1] / norm_accel;
        accel_norm[2] = accel[2] / norm_accel;
    } else {
        accel_norm[0] = 0.0f;
        accel_norm[1] = 0.0f;
        accel_norm[2] = 1.0f;  // 默认向下
    }
    
    // 虚拟磁力计数据（用于维持EKF结构，但我们主要依赖加速度计）
    // 在没有实际磁力计的情况下，我们可以设一个固定参考方向
    float magn_norm[3] = {1.0f, 0.0f, 0.0f};  // 北方向
    
    // 测量噪声协方差
    // R[0-2]: 加速度计噪声
    // R[3-5]: 磁力计噪声（由于没有真实磁力计，设大一些）
    // R[6-9]: 四元数噪声
    float R[10] = {
        0.1f, 0.1f, 0.1f,       // 加速度计噪声（根据需要调整）
        10.0f, 10.0f, 10.0f,    // 磁力计噪声（设大以降低影响）
        0.001f, 0.001f, 0.001f, 0.001f  // 四元数噪声
    };
    
    // 预测步骤：使用陀螺仪数据
    ekf13->Process(gyro, dt);
    
    // 更新步骤：使用加速度计和虚拟磁力计数据
    // 使用UpdateRefMeasurement而不是UpdateRefMeasurementMagn
    // 因为UpdateRefMeasurement更适合只有加速度计的情况
    ekf13->UpdateRefMeasurement(accel_norm, magn_norm, R);
    
    // 归一化四元数
    float q_norm = sqrt(ekf13->X(0,0)*ekf13->X(0,0) + 
                       ekf13->X(0,1)*ekf13->X(0,1) + 
                       ekf13->X(0,2)*ekf13->X(0,2) + 
                       ekf13->X(0,3)*ekf13->X(0,3));
    
    if (q_norm > 0.001f) {
        ekf13->X(0,0) /= q_norm;
        ekf13->X(0,1) /= q_norm;
        ekf13->X(0,2) /= q_norm;
        ekf13->X(0,3) /= q_norm;
    }
    
    // 将四元数转换为欧拉角（roll, pitch, yaw）
    float q0 = ekf13->X(0,0);
    float q1 = ekf13->X(0,1);
    float q2 = ekf13->X(0,2);
    float q3 = ekf13->X(0,3);
    
    // 横滚角 (x轴旋转)
    float sinr_cosp = 2.0f * (q0 * q1 + q2 * q3);
    float cosr_cosp = 1.0f - 2.0f * (q1 * q1 + q2 * q2);
    result.roll = atan2f(sinr_cosp, cosr_cosp) * 180.0f / M_PI;
    
    // 俯仰角 (y轴旋转)
    float sinp = 2.0f * (q0 * q2 - q3 * q1);
    if (fabs(sinp) >= 1.0f) {
        result.pitch = copysignf(M_PI / 2.0f, sinp) * 180.0f / M_PI;
    } else {
        result.pitch = asinf(sinp) * 180.0f / M_PI;
    }
    
    // 偏航角 (z轴旋转) - 没有磁力计时会漂移
    float siny_cosp = 2.0f * (q0 * q3 + q1 * q2);
    float cosy_cosp = 1.0f - 2.0f * (q2 * q2 + q3 * q3);
    result.yaw = atan2f(siny_cosp, cosy_cosp) * 180.0f / M_PI;
    
    return result;
}

void EKF_MPU6050::getQuaternion(float q[4]) {
    if (ekf13 && initialized) {
        q[0] = ekf13->X(0,0);
        q[1] = ekf13->X(0,1);
        q[2] = ekf13->X(0,2);
        q[3] = ekf13->X(0,3);
    }
}

void EKF_MPU6050::getGyroBias(float bias[3]) {
    if (ekf13 && initialized) {
        bias[0] = ekf13->X(0,4);  // 陀螺仪x轴偏差
        bias[1] = ekf13->X(0,5);  // 陀螺仪y轴偏差
        bias[2] = ekf13->X(0,6);  // 陀螺仪z轴偏差
    }
}