#include "init.hpp"

// ================== 任务函数 ==================
void task_mpu6050GetParam(void *pvParameter)
{
    while (1)
    {
        // 读取原始传感器数据
        mpu6050_get_acce(mpu6050, &mpu6050_acce);
        mpu6050_get_gyro(mpu6050, &mpu6050_gyro);
        mpu6050_get_temp(mpu6050, &mpu6050_temp);
        mpu6050_complimentory_filter(mpu6050, &mpu6050_acce, &mpu6050_gyro, &mpu6050_angle);
        ESP_LOGI("MPU6050", "Roll: %.3f°, Pitch: %.3f°", mpu6050_angle.roll, mpu6050_angle.pitch);
        vTaskDelay(pdMS_TO_TICKS(20)); // 建议 ≤50ms，滤波需要高频采样
    }
}
void task_oledDisplay_mpu6050(void *pvParameter)
{
    char rollStr[32];
    char pitchStr[32];
    while (1)
    {
        //     ssd1306_set_contrast(dev_hdl, 0xff);
        //     ssd1306_display_text_x2(dev_hdl, 0, "MPU6050", true);
        //     ssd1306_display_text(dev_hdl, 2, "test!!!", false);

        snprintf(rollStr, sizeof(rollStr), "Roll: %.2f", mpu6050_angle.roll);
        snprintf(pitchStr, sizeof(pitchStr), "Pitch: %.2f", mpu6050_angle.pitch);

        // ssd1306_display_text(dev_hdl, 3, rollStr, false);
        // ssd1306_display_text(dev_hdl, 4, pitchStr, false);
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

void task_ssd1306_animator(void *pvParameters)
{
}