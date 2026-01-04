#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "mpu6050.h"
#include "esp_log.h"
#include "ssd1306.h"

// ================== 配置区域 ==================

#define I2C_MASTER_SCL_IO 22                   // I2C 时钟线 SCL 连接到 GPIO22
#define I2C_MASTER_SDA_IO 21                   // I2C 数据线 SDA 连接到 GPIO21
#define I2C_MASTER_NUM I2C_NUM_0               // 使用 I2C 控制器 0
#define OLED_I2C_FREQ_HZ 400000                // I2C 通信频率：400kHz（标准高速模式）
#define OLED_I2C_ADDR 0x3C                     // 0x3C 是 SH1106 最常见的 7 位 I²C 地址；
#define MPU6050_I2C_ADDRESS 0x68u              /*MPU6050的默认I2C地址（当AD0引脚接地时为0x68）*/
#define MPU6050_I2C_ADDRESS_1 0x69u            /*当AD0引脚接高电平时的I2C地址（0x69）*/
#define MPU6050_WHO_AM_I_VAL 0x68u             // 用于验证设备型号的寄存器值（MPU6050的WHO_AM_I寄存器值为0x68）
static i2c_master_bus_handle_t i2c_bus = NULL; // 总线句柄
static mpu6050_handle_t mpu6050 = NULL;
ssd1306_config_t dev_cfg = I2C_SSD1306_128x64_CONFIG_DEFAULT;
ssd1306_handle_t dev_hdl;

// ================== 全局状态 ==================

// 从 ESP-IDF 5.0 开始，I²C 要先“安装总线”拿到一条
// i2c_master_bus_handle_t，再往这条总线上“挂设备”
static void i2c_bus_init(void)
{
    i2c_master_bus_config_t bus_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_MASTER_NUM,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_new_master_bus(&bus_cfg, &i2c_bus);
}

static void i2c_sensor_mpu6050_init(void)
{
    mpu6050 = mpu6050_create(i2c_bus, MPU6050_I2C_ADDRESS);
    mpu6050_config(mpu6050, ACCE_FS_4G, GYRO_FS_500DPS);
    mpu6050_wake_up(mpu6050);
}

static void i2c_sensor_ssd1306_init(void)
{
    ssd1306_init(i2c_bus, &dev_cfg, &dev_hdl);
    if (dev_hdl == NULL)
    {
        ESP_LOGE("OLED", "ssd1306 handle init failed"); // 如果初始化失败，记录错误日志
        assert(dev_hdl);                                // 确保设备句柄不为空
    }
}