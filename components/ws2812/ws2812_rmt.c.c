#include "ws2812_rmt.h"
#include <math.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/rmt_tx.h"
#include "esp_check.h"

static const char *TAG = "WS2812";

// 全局变量
static rmt_channel_handle_t led_chan = NULL;
static rmt_encoder_handle_t led_encoder = NULL;
static uint8_t led_rgb[3] = {0, 0, 0}; // GRB格式

// WS2812时序参数（单位：10MHz ticks，每个tick=0.1μs）
#define T0H_NS 350 // 0码高电平时间 350ns = 3.5 ticks
#define T0L_NS 700 // 0码低电平时间 700ns = 7 ticks
#define T1H_NS 700 // 1码高电平时间 700ns = 7 ticks
#define T1L_NS 350 // 1码低电平时间 350ns = 3.5 ticks

// 自定义编码器结构
typedef struct
{
    rmt_encoder_t base;
    rmt_encoder_t *bytes_encoder;
    rmt_encoder_t *copy_encoder;
    int state;
    rmt_symbol_word_t reset_symbol;
} rmt_ws2812_encoder_t;

// 编码器函数
static size_t rmt_encode_ws2812(rmt_encoder_t *encoder, rmt_channel_handle_t channel,
                                const void *primary_data, size_t data_size, rmt_encode_state_t *ret_state)
{
    rmt_ws2812_encoder_t *ws2812_encoder = __containerof(encoder, rmt_ws2812_encoder_t, base);
    rmt_encode_state_t session_state = 0;
    size_t encoded_symbols = 0;

    switch (ws2812_encoder->state)
    {
    case 0: // 发送RGB数据
        encoded_symbols += ws2812_encoder->bytes_encoder->encode(ws2812_encoder->bytes_encoder, channel,
                                                                 primary_data, data_size, &session_state);
        if (session_state & RMT_ENCODING_COMPLETE)
        {
            ws2812_encoder->state = 1; // 切换到复位状态
        }
        if (session_state & RMT_ENCODING_MEM_FULL)
        {
            *ret_state = RMT_ENCODING_MEM_FULL;
            return encoded_symbols;
        }
        // 继续发送复位信号
    case 1: // 发送复位信号（50μs低电平）
        encoded_symbols += ws2812_encoder->copy_encoder->encode(ws2812_encoder->copy_encoder, channel,
                                                                &ws2812_encoder->reset_symbol,
                                                                sizeof(rmt_symbol_word_t), &session_state);
        if (session_state & RMT_ENCODING_COMPLETE)
        {
            ws2812_encoder->state = 0; // 回到初始状态
            *ret_state = RMT_ENCODING_COMPLETE;
        }
        if (session_state & RMT_ENCODING_MEM_FULL)
        {
            *ret_state = RMT_ENCODING_MEM_FULL;
        }
    }

    return encoded_symbols;
}

static esp_err_t rmt_del_ws2812_encoder(rmt_encoder_t *encoder)
{
    rmt_ws2812_encoder_t *ws2812_encoder = __containerof(encoder, rmt_ws2812_encoder_t, base);
    rmt_del_encoder(ws2812_encoder->bytes_encoder);
    rmt_del_encoder(ws2812_encoder->copy_encoder);
    free(ws2812_encoder);
    return ESP_OK;
}

static esp_err_t rmt_ws2812_encoder_reset(rmt_encoder_t *encoder)
{
    rmt_ws2812_encoder_t *ws2812_encoder = __containerof(encoder, rmt_ws2812_encoder_t, base);
    rmt_encoder_reset(ws2812_encoder->bytes_encoder);
    rmt_encoder_reset(ws2812_encoder->copy_encoder);
    ws2812_encoder->state = 0;
    return ESP_OK;
}

// 创建WS2812编码器
static esp_err_t rmt_new_ws2812_encoder(rmt_encoder_handle_t *ret_encoder)
{
    esp_err_t ret = ESP_OK;
    rmt_ws2812_encoder_t *ws2812_encoder = NULL;

    // 分配编码器内存
    ws2812_encoder = calloc(1, sizeof(rmt_ws2812_encoder_t));
    if (ws2812_encoder == NULL)
    {
        ESP_LOGE(TAG, "分配编码器内存失败");
        return ESP_ERR_NO_MEM;
    }

    // 设置编码器函数
    ws2812_encoder->base.encode = rmt_encode_ws2812;
    ws2812_encoder->base.del = rmt_del_ws2812_encoder;
    ws2812_encoder->base.reset = rmt_ws2812_encoder_reset;
    ws2812_encoder->state = 0;

    // 设置复位信号（50μs低电平）
    ws2812_encoder->reset_symbol = (rmt_symbol_word_t){
        .level0 = 0,
        .duration0 = 500, // 50μs = 500 ticks @ 10MHz
        .level1 = 0,
        .duration1 = 0,
    };

    // 创建字节编码器
    rmt_bytes_encoder_config_t bytes_encoder_config = {
        .bit0 = {
            .level0 = 1,
            .duration0 = T0H_NS / 100, // 转换为10MHz ticks
            .level1 = 0,
            .duration1 = T0L_NS / 100,
        },
        .bit1 = {
            .level0 = 1,
            .duration0 = T1H_NS / 100,
            .level1 = 0,
            .duration1 = T1L_NS / 100,
        },
        .flags = {
            .msb_first = 1 // WS2812使用MSB优先
        }};

    ret = rmt_new_bytes_encoder(&bytes_encoder_config, &ws2812_encoder->bytes_encoder);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "创建字节编码器失败");
        free(ws2812_encoder);
        return ret;
    }

    // 创建复制编码器
    rmt_copy_encoder_config_t copy_encoder_config = {};
    ret = rmt_new_copy_encoder(&copy_encoder_config, &ws2812_encoder->copy_encoder);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "创建复制编码器失败");
        rmt_del_encoder(ws2812_encoder->bytes_encoder);
        free(ws2812_encoder);
        return ret;
    }

    *ret_encoder = &ws2812_encoder->base;
    return ESP_OK;
}

// 发送数据到WS2812
static void ws2812_send(void)
{
    if (led_chan == NULL || led_encoder == NULL)
    {
        ESP_LOGE(TAG, "WS2812未初始化");
        return;
    }

    rmt_transmit_config_t tx_config = {
        .loop_count = 0,
        .flags = {
            .eot_level = 0}};

    esp_err_t ret = rmt_transmit(led_chan, led_encoder, led_rgb, sizeof(led_rgb), &tx_config);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "发送数据失败: %s", esp_err_to_name(ret));
        return;
    }

    // 等待发送完成
    rmt_tx_wait_all_done(led_chan, pdMS_TO_TICKS(100));
}

// 初始化WS2812
void ws2812_init(int gpio_num)
{
    esp_err_t ret;

    ESP_LOGI(TAG, "初始化WS2812，GPIO: %d", gpio_num);

    // RMT通道配置
    rmt_tx_channel_config_t tx_chan_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .gpio_num = gpio_num,
        .mem_block_symbols = 64,
        .resolution_hz = 10000000, // 10MHz，每个tick=0.1μs
        .trans_queue_depth = 4,
        .flags = {
            .invert_out = false,
            .with_dma = false,
            .io_loop_back = false,
            .io_od_mode = false}};

    // 创建RMT通道
    ret = rmt_new_tx_channel(&tx_chan_config, &led_chan);
    ESP_ERROR_CHECK(ret);

    // 创建WS2812编码器
    ret = rmt_new_ws2812_encoder(&led_encoder);
    ESP_ERROR_CHECK(ret);

    // 启用RMT通道
    ret = rmt_enable(led_chan);
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "WS2812初始化完成");
}

// 设置LED颜色（GRB格式）
void ws2812_set_color(uint8_t r, uint8_t g, uint8_t b)
{
    // WS2812使用GRB顺序
    led_rgb[0] = g; // G
    led_rgb[1] = r; // R
    led_rgb[2] = b; // B

    ws2812_send();
}

// 清空LED
void ws2812_clear(void)
{
    memset(led_rgb, 0, sizeof(led_rgb));
    ws2812_send();
    vTaskDelay(pdMS_TO_TICKS(10));
}

// HSV转RGB函数
static void hsv_to_rgb(float h, float s, float v, uint8_t *r, uint8_t *g, uint8_t *b)
{
    float c = v * s;
    float x = c * (1 - fabs(fmod(h / 60.0, 2) - 1));
    float m = v - c;

    float r_f, g_f, b_f;

    if (h >= 0 && h < 60)
    {
        r_f = c;
        g_f = x;
        b_f = 0;
    }
    else if (h >= 60 && h < 120)
    {
        r_f = x;
        g_f = c;
        b_f = 0;
    }
    else if (h >= 120 && h < 180)
    {
        r_f = 0;
        g_f = c;
        b_f = x;
    }
    else if (h >= 180 && h < 240)
    {
        r_f = 0;
        g_f = x;
        b_f = c;
    }
    else if (h >= 240 && h < 300)
    {
        r_f = x;
        g_f = 0;
        b_f = c;
    }
    else
    {
        r_f = c;
        g_f = 0;
        b_f = x;
    }

    *r = (uint8_t)((r_f + m) * 255);
    *g = (uint8_t)((g_f + m) * 255);
    *b = (uint8_t)((b_f + m) * 255);
}

// 彩虹呼吸灯效果
void ws2812_rainbow_breathing(uint16_t period_ms)
{
    static uint32_t start_time = 0;
    static float hue = 0;

    if (start_time == 0)
    {
        start_time = xTaskGetTickCount();
    }

    // 计算经过的时间
    uint32_t current_time = xTaskGetTickCount();
    uint32_t elapsed = pdTICKS_TO_MS(current_time - start_time);

    // 色相变化（0-360度）
    hue = fmod(elapsed * 360.0 / period_ms, 360.0);

    // 亮度使用正弦波变化（0-1）
    float brightness = (sinf(elapsed * 2.0 * M_PI / period_ms) + 1.0) / 2.0;

    // 将HSV转换为RGB
    uint8_t r, g, b;
    hsv_to_rgb(hue, 1.0, brightness, &r, &g, &b);

    ws2812_set_color(r, g, b);

    // 控制刷新率（约60FPS）
    vTaskDelay(pdMS_TO_TICKS(16));
}

// 单色呼吸灯效果
void ws2812_color_breathing(uint8_t r, uint8_t g, uint8_t b, uint16_t period_ms)
{
    static uint32_t start_time = 0;

    if (start_time == 0)
    {
        start_time = xTaskGetTickCount();
    }

    // 计算经过的时间
    uint32_t current_time = xTaskGetTickCount();
    uint32_t elapsed = pdTICKS_TO_MS(current_time - start_time);

    // 亮度使用正弦波变化（0-1）
    float brightness = (sinf(elapsed * 2.0 * M_PI / period_ms) + 1.0) / 2.0;

    // 应用亮度到颜色
    uint8_t r_adj = (uint8_t)(r * brightness);
    uint8_t g_adj = (uint8_t)(g * brightness);
    uint8_t b_adj = (uint8_t)(b * brightness);

    ws2812_set_color(r_adj, g_adj, b_adj);

    // 控制刷新率（约60FPS）
    vTaskDelay(pdMS_TO_TICKS(16));
}

// 单色常亮
void ws2812_solid_color(uint8_t r, uint8_t g, uint8_t b)
{
    ws2812_set_color(r, g, b);
}