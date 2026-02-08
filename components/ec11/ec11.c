#include "ec11.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "freertos/task.h"
#include <stdlib.h>

#define TAG "EC11"

// EC11内部状态结构体
typedef struct ec11_t
{
    gpio_num_t pin_a;
    gpio_num_t pin_b;
    gpio_num_t pin_sw;

    bool active_level;      // 按键有效电平
    uint32_t debounce_us;   // 消抖时间（微秒）
    uint32_t long_press_us; // 长按时间（微秒）

    // 旋转编码器状态
    uint8_t last_state;      // 上一次AB状态（2位：A|B）
    uint64_t last_edge_time; // 上次边沿时间

    // 按键状态
    bool is_pressed;
    bool last_sw_state;
    uint64_t press_start_time;
    ec11_event_t pending_event;

    // 事件队列
    QueueHandle_t event_queue;
    bool isr_installed;
} ec11_t;

// 获取AB引脚状态（2位：A|B）
static inline uint8_t get_ab_state(ec11_handle_t handle)
{
    return (gpio_get_level(handle->pin_a) << 1) | gpio_get_level(handle->pin_b);
}

// 按键中断处理函数
static void IRAM_ATTR sw_isr_handler(void *arg)
{
    ec11_handle_t handle = (ec11_handle_t)arg;
    uint64_t current_time = esp_timer_get_time();
    bool current_state = gpio_get_level(handle->pin_sw);

    // 消抖处理
    if (current_time - handle->last_edge_time < handle->debounce_us)
    {
        return;
    }

    if (current_state != handle->last_sw_state)
    {
        handle->last_sw_state = current_state;
        handle->last_edge_time = current_time;

        // 按键按下事件
        if (current_state == handle->active_level)
        {
            handle->is_pressed = true;
            handle->press_start_time = current_time;
            handle->pending_event = EC11_EVENT_NONE;
        }
        // 按键释放事件
        else
        {
            handle->is_pressed = false;
            uint32_t press_duration = (current_time - handle->press_start_time) / 1000; // 毫秒

            // 判断长按/短按
            if (press_duration >= (handle->long_press_us / 1000))
            {
                handle->pending_event = EC11_EVENT_LONG_PRESS;
            }
            else if (press_duration >= (handle->debounce_us / 1000))
            {
                handle->pending_event = EC11_EVENT_SHORT_PRESS;
            }

            // 发送释放事件
            ec11_event_t release_event = EC11_EVENT_RELEASED;
            xQueueSendFromISR(handle->event_queue, &release_event, NULL);
        }
    }
}

// 旋转编码器中断处理函数
static void IRAM_ATTR rotary_isr_handler(void *arg)
{
    ec11_handle_t handle = (ec11_handle_t)arg;
    uint64_t current_time = esp_timer_get_time();

    // 消抖处理
    if (current_time - handle->last_edge_time < handle->debounce_us)
    {
        return;
    }

    uint8_t current_state = get_ab_state(handle);

    // 状态机检测旋转方向
    // 状态顺序：00->01->11->10->00（顺时针）
    // 状态顺序：00->10->11->01->00（逆时针）
    static const int8_t state_transition[4][4] = {
        // 下一状态：00  01  10  11
        {0, 1, -1, 0}, // 当前状态：00
        {-1, 0, 0, 1}, // 当前状态：01
        {1, 0, 0, -1}, // 当前状态：10
        {0, -1, 1, 0}  // 当前状态：11
    };

    int8_t direction = state_transition[handle->last_state][current_state];

    if (direction != 0)
    {
        handle->last_state = current_state;
        handle->last_edge_time = current_time;

        ec11_event_t event = (direction == 1) ? EC11_EVENT_CW : EC11_EVENT_CCW;
        xQueueSendFromISR(handle->event_queue, &event, NULL);
    }
}

ec11_handle_t ec11_create(const ec11_config_t *config)
{
    ec11_handle_t handle = (ec11_handle_t)malloc(sizeof(ec11_t));
    if (!handle)
    {
        ESP_LOGE(TAG, "Failed to allocate memory for EC11");
        return NULL;
    }

    // 初始化结构体
    handle->pin_a = config->pin_a;
    handle->pin_b = config->pin_b;
    handle->pin_sw = config->pin_sw;
    handle->active_level = config->active_level;
    handle->debounce_us = config->debounce_ms * 1000;
    handle->long_press_us = config->long_press_ms * 1000;

    handle->last_state = 0;
    handle->last_edge_time = 0;
    handle->is_pressed = false;
    handle->last_sw_state = !config->active_level; // 初始化为非按下状态
    handle->press_start_time = 0;
    handle->pending_event = EC11_EVENT_NONE;
    handle->isr_installed = false;

    // 创建事件队列
    handle->event_queue = xQueueCreate(config->queue_size, sizeof(ec11_event_t));
    if (!handle->event_queue)
    {
        ESP_LOGE(TAG, "Failed to create event queue");
        free(handle);
        return NULL;
    }

    return handle;
}

esp_err_t ec11_init(ec11_handle_t handle)
{
    if (!handle)
    {
        return ESP_ERR_INVALID_ARG;
    }

    // 配置A、B引脚为上拉输入
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << handle->pin_a) | (1ULL << handle->pin_b),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE,
    };
    gpio_config(&io_conf);

    // 配置SW引脚
    io_conf.pin_bit_mask = (1ULL << handle->pin_sw);
    io_conf.intr_type = GPIO_INTR_ANYEDGE;
    gpio_config(&io_conf);

    // 读取初始状态
    handle->last_state = get_ab_state(handle);
    handle->last_sw_state = gpio_get_level(handle->pin_sw);

    ESP_LOGI(TAG, "EC11 initialized: A=%d, B=%d, SW=%d",
             handle->pin_a, handle->pin_b, handle->pin_sw);

    return ESP_OK;
}

esp_err_t ec11_start(ec11_handle_t handle)
{
    if (!handle)
    {
        return ESP_ERR_INVALID_ARG;
    }

    // 安装GPIO中断服务
    if (!handle->isr_installed)
    {
        esp_err_t err = gpio_install_isr_service(0);
        if (err != ESP_OK && err != ESP_ERR_INVALID_STATE)
        {
            ESP_LOGE(TAG, "Failed to install ISR service: %s", esp_err_to_name(err));
            return err;
        }
        handle->isr_installed = true;
    }

    // 配置A、B引脚中断
    gpio_isr_handler_add(handle->pin_a, rotary_isr_handler, handle);
    gpio_isr_handler_add(handle->pin_b, rotary_isr_handler, handle);

    // 配置SW引脚中断
    gpio_isr_handler_add(handle->pin_sw, sw_isr_handler, handle);

    ESP_LOGI(TAG, "EC11 started");
    return ESP_OK;
}

void ec11_stop(ec11_handle_t handle)
{
    if (!handle)
    {
        return;
    }

    // 移除中断处理
    gpio_isr_handler_remove(handle->pin_a);
    gpio_isr_handler_remove(handle->pin_b);
    gpio_isr_handler_remove(handle->pin_sw);

    // 清除事件队列
    xQueueReset(handle->event_queue);

    ESP_LOGI(TAG, "EC11 stopped");
}

bool ec11_wait_event(ec11_handle_t handle, ec11_event_t *event, TickType_t timeout)
{
    if (!handle || !event)
    {
        return false;
    }

    // 首先检查是否有待处理的按键事件
    if (handle->pending_event != EC11_EVENT_NONE)
    {
        *event = handle->pending_event;
        handle->pending_event = EC11_EVENT_NONE;
        return true;
    }

    // 从队列中获取事件
    return xQueueReceive(handle->event_queue, event, timeout) == pdTRUE;
}

bool ec11_try_get_event(ec11_handle_t handle, ec11_event_t *event)
{
    if (!handle || !event)
    {
        return false;
    }

    // 首先检查是否有待处理的按键事件
    if (handle->pending_event != EC11_EVENT_NONE)
    {
        *event = handle->pending_event;
        handle->pending_event = EC11_EVENT_NONE;
        return true;
    }

    // 从队列中尝试获取事件
    return xQueueReceive(handle->event_queue, event, 0) == pdTRUE;
}

bool ec11_is_pressed(ec11_handle_t handle)
{
    return handle ? handle->is_pressed : false;
}

void ec11_delete(ec11_handle_t handle)
{
    if (!handle)
    {
        return;
    }

    // 停止EC11
    ec11_stop(handle);

    // 删除事件队列
    if (handle->event_queue)
    {
        vQueueDelete(handle->event_queue);
    }

    // 卸载中断服务
    if (handle->isr_installed)
    {
        gpio_uninstall_isr_service();
    }

    free(handle);
    ESP_LOGI(TAG, "EC11 deleted");
}