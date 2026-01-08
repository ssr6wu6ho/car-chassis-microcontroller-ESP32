// SPDX-License-Identifier: MIT
/*
 * ssd1306_private.h - Internal definitions (not part of public API)
 * Copyright (c) 2025 Jonathan Wåhrenberg
 */

#pragma once

#include "ssd1306.h"

#include <driver/i2c_master.h>
#include <esp_err.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

    // Vtable struct
    typedef struct
    {
        esp_err_t (*send_cmd)(void *ctx, const uint8_t *cmd, size_t n);
        esp_err_t (*send_data)(void *ctx, const uint8_t *data, size_t n);
        esp_err_t (*reset)(void *ctx);
    } ssd1306_bus_vt_t;

    struct ssd1306_t
    {
        // 公共部分
        const ssd1306_font_t *font; // 当前字体
        uint8_t *fb;                // 帧缓冲区指针
        size_t fb_len;              // 帧缓冲区大小

        // 总线抽象
        void *bus_ctx;              // 总线上下文（I2C设备句柄等）
        const ssd1306_bus_vt_t *vt; // 总线虚函数表指针

        // 并发控制
        SemaphoreHandle_t lock; // 互斥锁，保护多线程访问

        // 显示参数
        uint16_t width;  // 显示宽度
        uint16_t height; // 显示高度

        // 脏矩形跟踪（用于局部刷新优化）
        uint16_t dx0, dy0, dx1, dy1; // 脏矩形边界
        bool dirty;                  // 脏标志（是否需要刷新）

        // 资源管理标志
        bool driver_owns_fb; // 驱动是否拥有帧缓冲区
        bool initialized;    // 是否已初始化
    };

    // I2C functions
    esp_err_t ssd1306_bind_i2c(i2c_master_bus_handle_t bus, struct ssd1306_t *d, i2c_port_num_t port,
                               uint8_t addr, gpio_num_t rst_gpio);
    esp_err_t ssd1306_unbind_i2c(struct ssd1306_t *d);

#ifdef __cplusplus
}
#endif
