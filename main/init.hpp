#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "mpu6050.h"
#include "ssd1306.h"
#include "bottom.h"
// 图标和字体文件
#include "ssd1306_bitmap_animator.h"
// ================== 配置区域 ==================
#define BOTTOM_LEFT_PIN 33
#define BOTTOM_RIGHT_PIN 32
#define RGB_PIN 27
#define I2C_MASTER_SCL_IO 22     // I2C 时钟线 SCL 连接到 GPIO22
#define I2C_MASTER_SDA_IO 21     // I2C 数据线 SDA 连接到 GPIO21
#define I2C_MASTER_NUM I2C_NUM_0 // 使用 I2C 控制器 0
#define MPU6050_I2C_ADDRESS 0x68u
#define SSD1306_I2C_ADDRESS 0x3C
static i2c_master_bus_handle_t i2c_bus = NULL; // 总线句柄
static mpu6050_handle_t mpu6050 = NULL;
static ssd1306_handle_t oled = NULL;
static bottom_handle_t left_bottom = NULL;
static bottom_handle_t right_bottom = NULL;

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
    // 已经尽力修改驱动函数了后续会像mpu6050一样传入简单的参数去修改
    // 先保证能跑就行
    ssd1306_config_t cfg = {
        .width = 128,
        .height = 64,
        .fb = NULL, // let driver allocate internally
        .fb_len = 0,
        // I2c
        .port = I2C_NUM_0,
        .addr = SSD1306_I2C_ADDRESS, // typical SSD1306 I2C address
        .rst_gpio = GPIO_NUM_NC,     // no reset pin
    };
    ssd1306_connect_i2c(i2c_bus, &cfg, &oled);
}

// 初始化所有按钮
static void bottom_init(void) {
    ESP_LOGI("bottomInit", "Initializing buttons...");
    
    // 创建左按钮实例
    left_bottom = button_create(BOTTOM_LEFT_PIN);
    if (left_bottom == NULL) {
        ESP_LOGE("bottomInit", "Failed to create left button");
        return;
    }
    button_init_single(left_bottom);
    
    // 创建右按钮实例
    right_bottom = button_create(BOTTOM_RIGHT_PIN);
    if (right_bottom == NULL) {
        ESP_LOGE("bottomInit", "Failed to create right button");
        button_delete(left_bottom);
        left_bottom = NULL;
        return;
    }
    button_init_single(right_bottom);
    
    ESP_LOGI("bottomInit", "Buttons initialized successfully");
}