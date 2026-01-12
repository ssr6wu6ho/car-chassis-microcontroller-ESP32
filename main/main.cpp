#include <stdio.h>
#include "task.hpp"

// ================== 主程序入口 ==================

extern "C" void app_main();

void app_main()
{
    // 初始化
    i2c_bus_init();
    mpu6050_ekf_init();
    i2c_sensor_ssd1306_init();
    // bottom_init();
    RGB_init();
    // // //任务函数
    // xTaskCreate(task_mpu6050GetParam, "mpu6050_task", 2048, NULL, 5, NULL);
    // xTaskCreate(bottom_driver_task, "oled_test_task", 2048, NULL, 5, NULL);

    xTaskCreate(task_mpu6050GetParam_EKF, "oled_test_task", 2048, NULL, 5, NULL);
    xTaskCreate(task_oled_display_fancy_ui_enhanced, "oled_test_task", 2048, NULL, 5, NULL);
    xTaskCreate(RGB_task, "oled_test_task", 2048, NULL, 5, NULL);
}