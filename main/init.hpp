#include <stdio.h>
#include <string.h>
#include "driver/i2c.h"
#include "mpu6050.h"

// ================== 配置区域 ==================

#define I2C_MASTER_SCL_IO 22        // I2C 时钟线 SCL 连接到 GPIO22
#define I2C_MASTER_SDA_IO 21        // I2C 数据线 SDA 连接到 GPIO21
#define I2C_MASTER_NUM I2C_NUM_0    // 使用 I2C 控制器 0
#define OLED_I2C_FREQ_HZ 400000     // I2C 通信频率：400kHz（标准高速模式）
#define OLED_I2C_ADDR 0x3C          // 0x3C 是 SH1106 最常见的 7 位 I²C 地址；
#define MPU6050_I2C_ADDRESS 0x68u   /*MPU6050的默认I2C地址（当AD0引脚接地时为0x68）*/
#define MPU6050_I2C_ADDRESS_1 0x69u /*当AD0引脚接高电平时的I2C地址（0x69）*/
#define MPU6050_WHO_AM_I_VAL 0x68u  // 用于验证设备型号的寄存器值（MPU6050的WHO_AM_I寄存器值为0x68）

static mpu6050_handle_t mpu6050 = NULL;
// ================== 全局状态 ==================
// 注意：volatile 确保多任务环境下编译器不优化该变量，保证实时可见性
volatile bool button_pressed = false; // true = 任一按钮被按下

// I2C 总线初始化
static void i2c_bus_init(void)
{
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = (gpio_num_t)I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = (gpio_num_t)I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = OLED_I2C_FREQ_HZ;
    conf.clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL;
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 256, 256, 0);
}

static void i2c_sensor_mpu6050_init(void)
{
    mpu6050 = mpu6050_create(I2C_MASTER_NUM, MPU6050_I2C_ADDRESS);
    mpu6050_config(mpu6050, ACCE_FS_4G, GYRO_FS_500DPS);
    mpu6050_wake_up(mpu6050);
}