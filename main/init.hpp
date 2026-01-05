#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "mpu6050.h"
#include "esp_log.h"
#include "ssd1306.h"
// 图标和字体文件
#include "ssd1306_bitmap_animator.h"
// ================== 配置区域 ==================
#define PWM_LIGHT_PIN 2
#define I2C_MASTER_SCL_IO 22                   // I2C 时钟线 SCL 连接到 GPIO22
#define I2C_MASTER_SDA_IO 21                   // I2C 数据线 SDA 连接到 GPIO21
#define I2C_MASTER_NUM I2C_NUM_0               // 使用 I2C 控制器 0
#define MPU6050_I2C_ADDRESS 0x68u              /*MPU6050的默认I2C地址（当AD0引脚接地时为0x68）*/

static i2c_master_bus_handle_t i2c_bus = NULL; // 总线句柄
static mpu6050_handle_t mpu6050 = NULL;

static ssd1306_handle_t oled = NULL;

// ================== 全局状态 ==================

mpu6050_acce_value_t mpu6050_acce;
mpu6050_gyro_value_t mpu6050_gyro;
complimentary_angle_t mpu6050_angle = {0}; // 初始化为0
mpu6050_temp_value_t mpu6050_temp;

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
    ssd1306_config_t cfg = {
        .width = 128,
        .height = 64,
        .fb = NULL, // let driver allocate internally
        .fb_len = 0,
        .iface.i2c =
            {
                .port = I2C_NUM_0,
                .addr = 0x3C,            // typical SSD1306 I2C address
                .rst_gpio = GPIO_NUM_NC, // no reset pin
            },
    };
    ssd1306_new_i2c(&cfg, &oled);

    int frame = 0;
    while (1)
    {
        ssd1306_draw_bitmap(oled, 0, 0, frames_eye[frame], FRAME_WIDTH, FRAME_HEIGHT);
        frame = (frame + 1) % FRAME_COUNT;
        vTaskDelay(7);
        ssd1306_display(oled);
    }
}