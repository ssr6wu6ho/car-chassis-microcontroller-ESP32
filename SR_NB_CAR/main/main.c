#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/i2c.h"

#include "mpu6050.h"

// ================== 配置区域 ==================
// LED 相关配置（使用 ESP32 的 LEDC PWM 模块驱动 GPIO2）
#define LED_GPIO_NUM 2                  // 板载 LED 连接的 GPIO 引脚（低电平点亮）
#define LEDC_TIMER LEDC_TIMER_0         // 使用 LEDC 定时器 0
#define LEDC_MODE LEDC_LOW_SPEED_MODE   // 选择低速模式（适用于非音频类 PWM）
#define LEDC_CHANNEL LEDC_CHANNEL_0     // 使用通道 0 输出 PWM 信号
#define LEDC_DUTY_RES LEDC_TIMER_13_BIT // 占空比分辨率：13 位（最大值 8191）
#define LEDC_FREQUENCY 4000             // PWM 频率：4 kHz（避免人耳可听噪声）

#define BUTTON_GPIO_R 34
#define BUTTON_GPIO_L 35
#define BUTTON_DEBOUNCE_MS 20 // 按钮硬件消抖时间（典型值 10~50ms）

#define I2C_MASTER_SCL_IO 22        // I2C 时钟线 SCL 连接到 GPIO22
#define I2C_MASTER_SDA_IO 21        // I2C 数据线 SDA 连接到 GPIO21
#define I2C_MASTER_NUM I2C_NUM_0    // 使用 I2C 控制器 0
#define OLED_I2C_FREQ_HZ 100000     // I2C 通信频率：400kHz（标准高速模式）
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

// ================== 任务函数 ==================
void mpu6050_task(void *pvParameter)
{
    mpu6050_acce_value_t acce;
    mpu6050_gyro_value_t gyro;
    complimentary_angle_t angle = {0}; // 初始化为0
    mpu6050_temp_value_t temp;

    while (1)
    {
        // 读取原始传感器数据
        mpu6050_get_acce(mpu6050, &acce);
        mpu6050_get_gyro(mpu6050, &gyro);
        mpu6050_get_temp(mpu6050, &temp);
        mpu6050_complimentory_filter(mpu6050, &acce, &gyro, &angle);
        ESP_LOGI("MPU6050", "Roll: %.3f°, Pitch: %.3f°", angle.roll, angle.pitch);
        vTaskDelay(pdMS_TO_TICKS(20)); // 建议 ≤50ms，滤波需要高频采样
    }
}

void led_control_task(void *pvParameter)
{
    // --- LEDC PWM 初始化 ---
    // 配置 LEDC 定时器：决定 PWM 基频和分辨率
    ledc_timer_config_t timer = {
        .speed_mode = LEDC_MODE,          // 工作模式（低速/高速）
        .duty_resolution = LEDC_DUTY_RES, // 占空比精度（13 位 → 0~8191）
        .timer_num = LEDC_TIMER,          // 绑定到定时器 0
        .freq_hz = LEDC_FREQUENCY,        // 输出频率 4kHz（足够高以消除闪烁）
        .clk_cfg = LEDC_AUTO_CLK,         // 自动选择时钟源（推荐）
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer)); // 应用定时器配置
    // 配置 LEDC 通道：将 PWM 信号映射到具体 GPIO
    ledc_channel_config_t channel = {
        .speed_mode = LEDC_MODE,  // 与定时器模式一致
        .channel = LEDC_CHANNEL,  // 使用通道 0
        .timer_sel = LEDC_TIMER,  // 关联到已配置的定时器
        .gpio_num = LED_GPIO_NUM, // 输出到 GPIO2
        .duty = 0,                // 初始占空比 = 0（对应低电平 → LED 亮）
        .hpoint = 0,              // 脉冲起始偏移（通常为 0）
    };
    ESP_ERROR_CHECK(ledc_channel_config(&channel)); // 应用通道配置
    while (1)
    {
        // 例如：根据 button_pressed 改变亮度
        if (button_pressed)
        {
            ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 4095); // 半亮（假设 max=8191）
            ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
        }
        else
        {
            ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 0); // 常亮
            ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void button_task(void *pvParameter)
{
    // --- 按钮 GPIO 初始化（虽然主要在 button_task 中使用，但此处初始化确保引脚安全）---
    // 注意：实际项目中建议只在一个任务中初始化 GPIO，避免重复配置
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BUTTON_GPIO_R) | (1ULL << BUTTON_GPIO_L), // 选中两个按钮引脚
        .mode = GPIO_MODE_INPUT,                                           // 设置为输入模式
        .pull_up_en = GPIO_PULLUP_ENABLE,                                  // 启用内部上拉电阻（按钮按下时接地 → 读 0）
        .pull_down_en = GPIO_PULLDOWN_DISABLE,                             // 禁用下拉（避免与上拉冲突）
        .intr_type = GPIO_INTR_DISABLE,                                    // 不使用中断，采用轮询方式检测
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf)); // 应用 GPIO 配置

    while (1)
    {
        bool pressed = (gpio_get_level(BUTTON_GPIO_R) == 0) || (gpio_get_level(BUTTON_GPIO_L) == 0);

        if (pressed)
        {
            // 消抖确认
            vTaskDelay(pdMS_TO_TICKS(BUTTON_DEBOUNCE_MS));
            if ((gpio_get_level(BUTTON_GPIO_R) == 0) ||
                (gpio_get_level(BUTTON_GPIO_L) == 0))
            {
                button_pressed = true;
                // 等待所有按钮释放
                while ((gpio_get_level(BUTTON_GPIO_R) == 0) || (gpio_get_level(BUTTON_GPIO_L) == 0))
                {
                    vTaskDelay(pdMS_TO_TICKS(10));
                }
                // 消抖释放确认
                vTaskDelay(pdMS_TO_TICKS(BUTTON_DEBOUNCE_MS));
                button_pressed = false;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}


// ================== 主程序入口 ==================
void app_main(void)
{
    // 初始化 I2C 总线
    i2c_bus_init();
    i2c_sensor_mpu6050_init();
    xTaskCreate(mpu6050_task, "mpu6050_task", 2048, NULL, 5, NULL);
}