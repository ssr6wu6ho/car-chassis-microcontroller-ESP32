#ifndef EC11_H
#define EC11_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"          // 必需：提供 esp_err_t 定义
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

/**
 * EC11 事件类型
 */
typedef enum {
    EC11_EVENT_NONE = 0,      ///< 无事件
    EC11_EVENT_CW,            ///< 顺时针旋转
    EC11_EVENT_CCW,           ///< 逆时针旋转
    EC11_EVENT_SHORT_PRESS,   ///< 短按
    EC11_EVENT_LONG_PRESS,    ///< 长按
    EC11_EVENT_RELEASED,      ///< 释放
} ec11_event_t;

/**
 * EC11 句柄类型（不透明指针）
 */
typedef struct ec11_t* ec11_handle_t;

/**
 * EC11 配置结构体
 */
typedef struct {
    uint8_t pin_a;            ///< CLK 引脚 (A 相)
    uint8_t pin_b;            ///< DT 引脚 (B 相)
    uint8_t pin_sw;           ///< SW 按键引脚
    bool active_level;        ///< 按键有效电平 (true: 高有效, false: 低有效)
    uint32_t debounce_ms;     ///< 消抖时间 (毫秒)
    uint32_t long_press_ms;   ///< 长按判定时间 (毫秒)
    uint32_t queue_size;      ///< 事件队列大小
} ec11_config_t;

/**
 * @brief 创建 EC11 句柄
 * @param config 配置参数指针
 * @return 成功返回句柄，失败返回 NULL
 */
ec11_handle_t ec11_create(const ec11_config_t* config);

/**
 * @brief 初始化 EC11 硬件 GPIO
 * @param handle EC11 句柄
 * @return ESP_OK 成功，其他为错误码
 */
esp_err_t ec11_init(ec11_handle_t handle);

/**
 * @brief 启动 EC11（注册中断）
 * @param handle EC11 句柄
 * @return ESP_OK 成功，其他为错误码
 */
esp_err_t ec11_start(ec11_handle_t handle);

/**
 * @brief 停止 EC11（移除中断）
 * @param handle EC11 句柄
 */
void ec11_stop(ec11_handle_t handle);

/**
 * @brief 阻塞式获取事件
 * @param handle EC11 句柄
 * @param event 事件输出指针
 * @param timeout 超时时间 (RTOS ticks)
 * @return true: 获取成功, false: 超时
 */
bool ec11_wait_event(ec11_handle_t handle, ec11_event_t* event, TickType_t timeout);

/**
 * @brief 非阻塞式获取事件
 * @param handle EC11 句柄
 * @param event 事件输出指针
 * @return true: 获取成功, false: 无事件
 */
bool ec11_try_get_event(ec11_handle_t handle, ec11_event_t* event);

/**
 * @brief 查询按键当前物理状态
 * @param handle EC11 句柄
 * @return true: 按下, false: 释放
 */
bool ec11_is_pressed(ec11_handle_t handle);

/**
 * @brief 释放 EC11 资源
 * @param handle EC11 句柄
 */
void ec11_delete(ec11_handle_t handle);

#ifdef __cplusplus
}
#endif

#endif // EC11_H