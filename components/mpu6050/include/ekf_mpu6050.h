#ifndef EKF_MPU6050_H
#define EKF_MPU6050_H

#include "esp_dsp.h"
#include "ekf_imu13states.h"

class EKF_MPU6050 {
private:
    ekf_imu13states *ekf13;
    bool initialized;
    float last_time_us;
    float dt;
    
public:
    struct Attitude {
        float roll;     // 横滚角 (°)
        float pitch;    // 俯仰角 (°)
        float yaw;      // 偏航角 (°)
    };
    
    EKF_MPU6050();
    ~EKF_MPU6050();
    
    // 初始化卡尔曼滤波器
    void init();
    
    // 更新IMU数据并计算姿态
    // accel: 加速度计数据 [x, y, z] (单位: g 或 m/s²)
    // gyro: 陀螺仪数据 [x, y, z] (单位: rad/s 或 °/s)
    // dt: 时间间隔 (秒)，如果为0则自动计算
    // 返回: 欧拉角姿态 (roll, pitch, yaw)
    Attitude update(float accel[3], float gyro[3], float dt = 0.0f);
    
    // 重置滤波器状态
    void reset();
    
    // 获取当前四元数
    void getQuaternion(float q[4]);
    
    // 获取估计的陀螺仪偏差
    void getGyroBias(float bias[3]);
};

#endif // EKF_MPU6050_H