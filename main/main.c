#include <stdio.h>
#include "task.hpp"

// ================== 主程序入口 ==================
void app_main(void)
{
    // 初始化
    i2c_bus_init();
    i2c_sensor_mpu6050_init();
    i2c_sensor_ssd1306_init();
    bottom_init();
    // // //任务函数
    xTaskCreate(task_mpu6050GetParam, "mpu6050_task", 2048, NULL, 5, NULL);
    // xTaskCreate(task_oledDisplay_mpu6050, "oled_test_task", 2048, NULL, 5, NULL);
    // xTaskCreate(task_ssd1306_animator, "oled_test_task", 2048, NULL, 5, NULL);
    xTaskCreate(bottom_driver_task, "oled_test_task", 2048, NULL, 5, NULL);
    xTaskCreate(task_oled_display_fancy_ui_enhanced, "oled_test_task", 2048, NULL, 5, NULL);
}