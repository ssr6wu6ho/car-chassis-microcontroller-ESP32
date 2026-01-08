#include "bottom.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// 配置参数
#define DEBOUNCE_TIME_MS 20     // 消抖时间
#define LONG_PRESS_TIME_MS 1000 // 长按阈值

// 创建按钮实例
bottom_handle_t button_create(uint8_t pin)
{
    bottom_handle_t btn = (bottom_handle_t)malloc(sizeof(bottom_t));
    if (btn == NULL)
    {
        ESP_LOGE("BUTTON", "Failed to allocate memory for button");
        return NULL;
    }

    btn->pin = pin;
    btn->is_pressed = false;
    btn->last_state = true; // 假设默认是高电平（未按下）
    btn->press_start_time = 0;
    btn->pending_event = BUTTON_EVENT_NONE;

    return btn;
}

// 初始化按钮硬件
void button_init_single(bottom_handle_t btn)
{
    if (btn == NULL)
        return;

    // 配置GPIO为上拉输入
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << btn->pin),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = (btn->pin >= 34 && btn->pin <= 39) ? GPIO_PULLUP_DISABLE : GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);

    // 读取初始状态
    btn->last_state = gpio_get_level(btn->pin);

    ESP_LOGI("BUTTON", "Button on pin %d initialized", btn->pin);
}

// 更新按钮状态
void button_update(bottom_handle_t btn)
{
    if (btn == NULL)
        return;

    uint64_t current_time = esp_timer_get_time() / 1000; // 毫秒
    bool current_state = gpio_get_level(btn->pin);

    // 检测状态变化
    if (current_state != btn->last_state)
    {
        // 消抖处理 - 延迟读取
        vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_TIME_MS));
        current_state = gpio_get_level(btn->pin);

        // 如果消抖后状态仍与上次不同，则状态确实变化了
        if (current_state != btn->last_state)
        {
            btn->last_state = current_state;

            // 检测按下事件（假设按下为低电平）
            if (current_state == 0 && !btn->is_pressed)
            {
                btn->is_pressed = true;
                btn->press_start_time = current_time;
                btn->pending_event = BUTTON_EVENT_NONE; // 清除旧事件
            }
            // 检测释放事件
            else if (current_state == 1 && btn->is_pressed)
            {
                btn->is_pressed = false;
                uint32_t press_duration = current_time - btn->press_start_time;

                // 判断短按还是长按
                if (press_duration >= LONG_PRESS_TIME_MS)
                {
                    btn->pending_event = BUTTON_EVENT_LONG_PRESS;
                }
                else if (press_duration >= DEBOUNCE_TIME_MS)
                {
                    btn->pending_event = BUTTON_EVENT_SHORT_PRESS;
                }
            }
        }
    }

    // 长按检测（持续按下）
    if (btn->is_pressed)
    {
        uint32_t press_duration = current_time - btn->press_start_time;

        // 如果达到长按时间且还没有触发事件
        if (press_duration >= LONG_PRESS_TIME_MS && btn->pending_event == BUTTON_EVENT_NONE)
        {
            btn->pending_event = BUTTON_EVENT_LONG_PRESS;
        }
    }
}

// 获取按钮事件
button_event_t button_get_event(bottom_handle_t btn)
{
    if (btn == NULL)
        return BUTTON_EVENT_NONE;

    button_event_t event = btn->pending_event;
    btn->pending_event = BUTTON_EVENT_NONE; // 读取后清除
    return event;
}

// 获取按钮是否按下
bool button_is_pressed(bottom_handle_t btn)
{
    if (btn == NULL)
        return false;
    return btn->is_pressed;
}

// 释放按钮资源
void button_delete(bottom_handle_t btn)
{
    if (btn != NULL)
    {
        free(btn);
    }
}