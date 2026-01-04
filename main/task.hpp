#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include "esp_log.h"
#include"init.hpp"

// ================== 任务函数 ==================
void mpu6050_task(void *pvParameter)
{
    mpu6050_acce_value_t acce;
    mpu6050_gyro_value_t gyro;
    complimentary_angle_t angle = {0}; // 初始化为0
    mpu6050_temp_value_t temp;

    while (1)
    {
        // 读取原始传感器数据
        mpu6050_get_acce(mpu6050, &acce);
        mpu6050_get_gyro(mpu6050, &gyro);
        mpu6050_get_temp(mpu6050, &temp);
        mpu6050_complimentory_filter(mpu6050, &acce, &gyro, &angle);
        ESP_LOGI("MPU6050", "Roll: %.3f°, Pitch: %.3f°", angle.roll, angle.pitch);
        vTaskDelay(pdMS_TO_TICKS(20)); // 建议 ≤50ms，滤波需要高频采样
    }
}
