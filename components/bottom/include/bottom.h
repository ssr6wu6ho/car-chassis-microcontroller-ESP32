#ifndef BUTTON_H
#define BUTTON_H

#include <stdbool.h>
#include <stdint.h>

// 事件类型
typedef enum {
    BUTTON_EVENT_NONE = 0,
    BUTTON_EVENT_SHORT_PRESS,
    BUTTON_EVENT_LONG_PRESS,
    BUTTON_EVENT_RELEASED,
} button_event_t;

// 按钮状态结构体
typedef struct {
    uint8_t pin;                    // 引脚
    bool is_pressed;               // 是否按下
    bool last_state;               // 上次状态
    uint64_t press_start_time;     // 按下开始时间
    button_event_t pending_event;  // 待处理事件
} bottom_t;

// 句柄类型定义
typedef bottom_t* bottom_handle_t;

// 初始化单个按钮
bottom_handle_t button_create(uint8_t pin);

// 初始化按钮硬件
void button_init_single(bottom_handle_t btn);

// 更新按钮状态
void button_update(bottom_handle_t btn);

// 获取按钮事件
button_event_t button_get_event(bottom_handle_t btn);

// 获取按钮是否按下
bool button_is_pressed(bottom_handle_t btn);

// 释放按钮资源（如果动态分配）
void button_delete(bottom_handle_t btn);

#endif // BUTTON_H