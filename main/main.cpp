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
    // RGB_init();
    // spin_bottom_init();
    uart1_init();
    //uart2_init();

    xTaskCreate(task_mpu6050GetParam_EKF, "task_mpu6050GetParam_EKF", 2048, NULL, 5, NULL);
    xTaskCreate(task_oled_display_fancy_ui_enhanced, "task_oled_display_fancy_ui_enhanced", 2048, NULL, 5, NULL);
    // xTaskCreate(bottom_driver_task, "bottom_driver_task", 2048, NULL, 5, NULL);
    // xTaskCreate(RGB_task, "RGB_task", 2048, NULL, 5, NULL);
    // xTaskCreate(ec11_test_task, "ec11_test_task", 2048, NULL, 5, NULL);
    xTaskCreate(motor_control_task, "motor_task", 4096, NULL, 5, NULL);
    vTaskDelete(NULL);
}