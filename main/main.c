#include <stdio.h>
#include "task.hpp"

// ================== 主程序入口 ==================
void app_main(void)
{
    // 初始化
    i2c_bus_init();
    i2c_sensor_mpu6050_init();
    //任务函数
    xTaskCreate(mpu6050_task, "mpu6050_task", 2048, NULL, 5, NULL);
}