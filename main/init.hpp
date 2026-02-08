// init.hpp
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "mpu6050.h"
#include "ssd1306.h"
#include "bottom.h"
#include "ws2812_rmt.h"
#include "ekf_mpu6050.h"
#include "ec11.h"
#include "foc.h"
#include "iot_servo.h"
// 图标和字体文件
#include "ssd1306_bitmap_animator.h"
// ================== 配置区域 ==================
#define BOTTOM_LEFT_PIN GPIO_NUM_33
#define BOTTOM_RIGHT_PIN GPIO_NUM_32
#define RGB_PIN GPIO_NUM_48          // RGB引脚改为48
#define I2C_MASTER_SCL_IO GPIO_NUM_5 // I2C 时钟线 SCL 连接到 GPIO5
#define I2C_MASTER_SDA_IO GPIO_NUM_4 // I2C 数据线 SDA 连接到 GPIO4
#define I2C_MASTER_NUM I2C_NUM_0     // 使用 I2C 控制器 0
#define MPU6050_I2C_ADDRESS 0x68u
#define SSD1306_I2C_ADDRESS 0x3C
// EC11引脚定义
#define EC11_PIN_A_CLK GPIO_NUM_41
#define EC11_PIN_B_DT GPIO_NUM_40
#define EC11_PIN_SW GPIO_NUM_42
// 舵机引脚定义
#define SERVO_L_PIN GPIO_NUM_9
#define SERVO_R_PIN GPIO_NUM_10
// UART配置
#define UART_NUM UART_NUM_1
#define UART_BAUD 921600
#define UART_TX_PIN 17
#define UART_RX_PIN 18
#define UART_BUF_SIZE 256
// UART2 配置
#define UART2_NUM UART_NUM_2
#define UART2_BAUD 921600 // 可以根据需要调整
#define UART2_TX_PIN 15
#define UART2_RX_PIN 16
#define UART2_BUF_SIZE 256
// UART超时配置
#define CMD_TIMEOUT_MS 100 // 命令发送后等待响应时间
#define RX_TIMEOUT_MS 50   // 接收单个字符超时
// 全局变量用于存储UART句柄
static QueueHandle_t uart_queue = NULL;
static QueueHandle_t uart2_queue;
// 全局句柄
static i2c_master_bus_handle_t i2c_bus = NULL; // 总线句柄
static mpu6050_handle_t mpu6050 = NULL;
static ssd1306_handle_t oled = NULL;
static bottom_handle_t left_bottom = NULL;
static bottom_handle_t right_bottom = NULL;
static ec11_handle_t ec11_handle = NULL;
static EKF_MPU6050 ekf_filter;

// ================== 全局状态 ==================

mpu6050_acce_value_t mpu6050_acce;
mpu6050_gyro_value_t mpu6050_gyro;
complimentary_angle_t mpu6050_angle = {0, 0}; // 初始化为0
mpu6050_temp_value_t mpu6050_temp;

// 从 ESP-IDF 5.0 开始，I²C 要先“安装总线”拿到一条
// i2c_master_bus_handle_t，再往这条总线上“挂设备”
static esp_err_t i2c_bus_init(void)
{
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_MASTER_NUM, // 必须放在第一位
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,     // 添加默认值
        .trans_queue_depth = 0, // 添加默认值
        .flags = {
            .enable_internal_pullup = true,
            .allow_pd = 0, // 添加默认值
        },
    };
    // 检查判断
    esp_err_t ret = i2c_new_master_bus(&bus_cfg, &i2c_bus);
    if (ret != ESP_OK)
    {
        ESP_LOGE("I2C", "I2C bus initialization failed: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI("I2C", "I2C bus initialized successfully");
    return ESP_OK;
}

esp_err_t uart1_init(void)
{
    esp_err_t ret = ESP_OK;

    // 1. 配置UART参数
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_APB,
    };

    // 清理残留驱动（防御性编程：避免重复初始化冲突）
    uart_driver_delete(UART_NUM);

    // 3. 配置UART参数
    ret = uart_param_config(UART_NUM, &uart_config);
    if (ret != ESP_OK)
    {
        ESP_LOGE("uart", "Failed to config UART parameters: %s", esp_err_to_name(ret));
        uart_driver_delete(UART_NUM);
        return ret;
    }

    // 4. 设置UART引脚
    ret = uart_set_pin(UART_NUM,
                       UART_TX_PIN,
                       UART_RX_PIN,
                       UART_PIN_NO_CHANGE,  // RTS
                       UART_PIN_NO_CHANGE); // CTS
    if (ret != ESP_OK)
    {
        ESP_LOGE("uart", "Failed to set UART pins: %s", esp_err_to_name(ret));
        uart_driver_delete(UART_NUM);
        return ret;
    }

    // 2. 安装UART驱动
    ret = uart_driver_install(UART_NUM,
                              UART_BUF_SIZE * 2,
                              UART_BUF_SIZE * 2,
                              10,          // 队列大小
                              &uart_queue, // 事件队列
                              0);          // 不设置中断分配标志
    if (ret != ESP_OK)
    {
        ESP_LOGE("uart", "Failed to install UART driver: %s", esp_err_to_name(ret));
        return ret;
    }

    // 5. 设置接收超时
    uint16_t timeout_calc = (RX_TIMEOUT_MS * UART_BAUD) / 10000;
    timeout_calc = (timeout_calc > 127) ? 127 : (timeout_calc < 1 ? 1 : timeout_calc); // 限制0-127
    ret = uart_set_rx_timeout(UART_NUM, timeout_calc);                                 // 单位：字符时间（1字符=10位）

    // 6. 清空缓冲区
    uart_flush(UART_NUM);

    ESP_LOGI("uart", "UART1 initialized on UART%d (TX:%d, RX:%d, %d bps)",
             UART_NUM, UART_TX_PIN, UART_RX_PIN, UART_BAUD);

    return ESP_OK;

    // 设置电机uart
    foc_set_uart_port(UART_NUM);
}

esp_err_t uart2_init(void)
{
    esp_err_t ret = ESP_OK;

    // 配置UART参数
    uart_config_t uart_config = {
        .baud_rate = UART2_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_APB,
    };

    // 清理残留驱动
    uart_driver_delete(UART2_NUM);

    // 安装UART驱动
    ret = uart_driver_install(UART2_NUM,
                              UART2_BUF_SIZE * 2,
                              UART2_BUF_SIZE * 2,
                              10,
                              &uart2_queue,
                              0);
    if (ret != ESP_OK)
    {
        ESP_LOGE("uart2", "Failed to install driver: %s", esp_err_to_name(ret));
        return ret;
    }

    // 配置UART参数
    ret = uart_param_config(UART2_NUM, &uart_config);
    if (ret != ESP_OK)
    {
        ESP_LOGE("uart2", "Failed to config parameters: %s", esp_err_to_name(ret));
        uart_driver_delete(UART2_NUM);
        return ret;
    }

    // 设置UART引脚
    ret = uart_set_pin(UART2_NUM,
                       UART2_TX_PIN,
                       UART2_RX_PIN,
                       UART_PIN_NO_CHANGE,
                       UART_PIN_NO_CHANGE);
    if (ret != ESP_OK)
    {
        ESP_LOGE("uart2", "Failed to set pins: %s", esp_err_to_name(ret));
        uart_driver_delete(UART2_NUM);
        return ret;
    }
    // 清空缓冲区
    uart_flush(UART2_NUM);

    ESP_LOGI("uart2", "UART2 initialized (TX:%d, RX:%d, %d bps)",
             UART2_TX_PIN, UART2_RX_PIN, UART2_BAUD);

    return ESP_OK;
}
static void mpu6050_ekf_init(void)
{
    mpu6050 = mpu6050_create(i2c_bus, MPU6050_I2C_ADDRESS);
    mpu6050_config(mpu6050, ACCE_FS_4G, GYRO_FS_500DPS);
    mpu6050_wake_up(mpu6050);
    // 初始化EKF
    ekf_filter.init();
}
static void i2c_sensor_ssd1306_init(void)
{
    // 已经尽力修改驱动函数了后续会像mpu6050一样传入简单的参数去修改
    // 先保证能跑就行
    ssd1306_config_t cfg = {
        .fb = NULL, // 按照声明顺序，fb 必须放在第一位
        .fb_len = 0,
        .width = 128,
        .height = 64,
        .port = I2C_NUM_0,
        .rst_gpio = GPIO_NUM_NC,
        .addr = SSD1306_I2C_ADDRESS,
    };
    ssd1306_connect_i2c(i2c_bus, &cfg, &oled);
}
// 初始化所有按钮
static void bottom_init(void)
{
    // 创建左按钮实例
    left_bottom = button_create(BOTTOM_LEFT_PIN);
    if (left_bottom == NULL)
    {
        ESP_LOGE("bottomInit", "Failed to create left button");
        return;
    }
    button_init_single(left_bottom);

    // 创建右按钮实例
    right_bottom = button_create(BOTTOM_RIGHT_PIN);
    if (right_bottom == NULL)
    {
        ESP_LOGE("bottomInit", "Failed to create right button");
        button_delete(left_bottom);
        left_bottom = NULL;
        return;
    }
    button_init_single(right_bottom);

    ESP_LOGI("bottomInit", "Buttons initialized successfully");
}

void spin_bottom_init(void)
{
    ec11_config_t config = {
        .pin_a = EC11_PIN_A_CLK,
        .pin_b = EC11_PIN_B_DT,
        .pin_sw = EC11_PIN_SW,
        .active_level = false, // 假设按下为低电平
        .debounce_ms = 20,     // 20ms消抖时间
        .long_press_ms = 1000, // 1秒长按
        .queue_size = 10,      // 事件队列大小
    };
    // 创建EC11句柄
    ec11_handle = ec11_create(&config);
    // 初始化硬件
    ec11_init(ec11_handle);
    // 启动EC11
    ec11_start(ec11_handle);
    ESP_LOGI("EC11", "EC11 initialized successfully");
    ESP_LOGI("EC11", "Pins: A=%d, B=%d, SW=%d",
             EC11_PIN_A_CLK, EC11_PIN_B_DT, EC11_PIN_SW);
}

static void RGB_init(void)
{
    ws2812_init(RGB_PIN);
}

static void servo_init(void)
{
    servo_config_t servo_l_cfg = {
        .max_angle = 180,
        .min_width_us = 500,
        .max_width_us = 2500,
        .freq = 50,
        .timer_number = LEDC_TIMER_0,
        .channels = {
            .servo_pin = {
                SERVO_L_PIN,
            },
            .ch = {
                LEDC_CHANNEL_0,
            },
        },
        .channel_number = 1,
    };
    servo_config_t servo_r_cfg = {
        .max_angle = 180,
        .min_width_us = 500,
        .max_width_us = 2500,
        .freq = 50,
        .timer_number = LEDC_TIMER_0,
        .channels = {
            .servo_pin = {
                SERVO_L_PIN,
            },
            .ch = {
                LEDC_CHANNEL_0,
            },
        },
        .channel_number = 1,
    };
    iot_servo_init(LEDC_LOW_SPEED_MODE, &servo_r_cfg);
}
