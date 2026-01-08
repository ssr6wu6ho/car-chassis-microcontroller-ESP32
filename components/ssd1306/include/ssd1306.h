// SPDX-License-Identifier: MIT
/*
 * ssd1306.h - SSD1306 driver header
 * Copyright (c) 2025 Jonathan Wåhrenberg
 */

#pragma once

#include <driver/gpio.h>
#include <driver/i2c_types.h>
#include <esp_err.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @brief Bitmap font descriptor.
     *
     * Glyphs are stored column-major, one byte per column.
     * Bit0 = top pixel, bit(height-1) = bottom.
     */
    typedef struct
    {
        uint8_t width;         // 字符宽度（像素），例如5（5x7字体）
        uint8_t height;        // 字符高度（像素），例如7
        uint8_t first;         // 支持的起始ASCII码，例如32（空格）
        uint8_t last;          // 支持的结束ASCII码，例如126（~）
        const uint8_t *bitmap; // 指向字体位图数据的指针
    } ssd1306_font_t;

    /**
     * @brief 主配置结构
     */
    typedef struct
    {

        uint8_t *fb;     // 可选的帧缓冲区指针
        size_t fb_len;   // 帧缓冲区长度（字节）
        uint16_t width;  // 显示宽度（像素）
        uint16_t height; // 显示高度（像素）

        i2c_port_num_t port; // I2C端口号（如I2C_NUM_0）
        gpio_num_t rst_gpio; // 复位引脚（GPIO_NUM_NC表示不使用）
        uint8_t addr;        // 7位I2C地址（0x3C或0x3D）
    } ssd1306_config_t;

    /**
     * @brief 显示句柄
     */
    typedef struct ssd1306_t *ssd1306_handle_t;

    /**
     * @brief Create and initialize a new SSD1306 display on I2C.
     *
     * @param[in]  cfg Configuration parameters.
     * @param[out] out Returned display handle.
     * @return ESP_OK on success, error otherwise.
     */
    esp_err_t ssd1306_connect_i2c(i2c_master_bus_handle_t bus_handle, const ssd1306_config_t *cfg, ssd1306_handle_t *out);

    /**
     * @brief Set the active font for text drawing.
     *
     * @param h Display handle.
     * @param font Font descriptor to use.
     * @return ESP_OK on success.
     */
    esp_err_t ssd1306_set_font(ssd1306_handle_t h, const ssd1306_font_t *font);

    /**
     * @brief Delete a display handle and free associated resources.
     *
     * @param h Display handle.
     * @return ESP_OK on success.
     */
    esp_err_t ssd1306_del(ssd1306_handle_t h);

    // ----- Drawing API -----

    /**
     * @brief Clear the entire framebuffer (set all pixels off).
     *
     * @param h Display handle.
     * @return ESP_OK on success.
     */
    esp_err_t ssd1306_clear(ssd1306_handle_t h);

    /**
     * @brief Draw or clear a single pixel.
     *
     * @param h Display handle.
     * @param x X-coordinate
     * @param y Y-coordinate
     * @param on true to set pixel, false to clear.
     * @return ESP_OK on success.
     */
    esp_err_t ssd1306_draw_pixel(ssd1306_handle_t h, int x, int y, bool on);

    /**
     * @brief Draw a rectangle (filled or outline).
     *
     * @param h Display handle.
     * @param x Top left corner X-coordinate.
     * @param y Top left corner Y-coordinate.
     * @param w Rectangle width.
     * @param hgt Rectangle height.
     * @param fill true to fill, false to outline.
     * @return ESP_OK on success.
     */
    esp_err_t ssd1306_draw_rect(ssd1306_handle_t h, int x, int y, int w, int hgt,
                                bool fill);

    /**
     * @brief Draw a straight line between two points.
     *
     * @param h Display handle.
     * @param x0 Point 1 X-coordinate.
     * @param y0 Point 1 Y-coordinate.
     * @param x1 Point 2 X-coordinate.
     * @param y1 Point 2 Y-coordinate.
     * @param on true to set pixel, false to clear.
     * @return ESP_OK on success.
     */
    esp_err_t ssd1306_draw_line(ssd1306_handle_t h, int x0, int y0, int x1, int y1,
                                bool on);

    /**
     * @brief Draw a circle (filled or outline).
     *
     * @param h Display handle.
     * @param xc Center point X-coordinate.
     * @param yc Center point Y-coordinate.
     * @param r Circle radius.
     * @param fill true to fill, false to outline.
     * @return ESP_OK on success.
     */
    esp_err_t ssd1306_draw_circle(ssd1306_handle_t h, int xc, int yc, int r,
                                  bool fill);

    /**
     * @brief Draw ASCII text using the current font, scale = 1.
     *
     * @param h Display handle.
     * @param x Top left X-coordinate.
     * @param y Top left Y-coordinate.
     * @param text Text to be displayed.
     * @param on true to set pixel, false to clear.
     * @return ESP_OK on success.
     */
    esp_err_t ssd1306_draw_text(ssd1306_handle_t h, int x, int y, const char *text,
                                bool on);

    /**
     * @brief Draw ASCII text with specified integer scale factor.
     *
     * @param h Display handle.
     * @param x Top left X-coordinate.
     * @param y Top left Y-coordinate.
     * @param text Text to be displayed.
     * @param on true to set pixel, false to clear.
     * @param scale Integer scale factor.
     * @return ESP_OK on success.
     */
    esp_err_t ssd1306_draw_text_scaled(ssd1306_handle_t h, int x, int y,
                                       const char *txt, bool on, int scale);

    /**
     * @brief Draw ASCII text wrapped inside a rectangle.
     *
     * Word-wraps on spaces, falls back to character wrapping for overlong words,
     * and honors '\n' as an explicit line break. Uses the current font and scale=1
     * unless you pass a different scale.
     *
     * @param h     Display handle.
     * @param x,y   Top-left of the wrapping rectangle.
     * @param w,hgt Width and height of the wrapping rectangle (pixels).
     * @param text  NUL-terminated ASCII string.
     * @param on    true sets pixels, false clears them.
     * @return ESP_OK on success.
     */
    esp_err_t ssd1306_draw_text_wrapped(ssd1306_handle_t h, int x, int y, int w,
                                        int hgt, const char *text, bool on);
    /**
     * @brief Draw ASCII text wrapped inside a rectangle.
     *
     * Word-wraps on spaces, falls back to character wrapping for overlong words,
     * and honors '\n' as an explicit line break. Uses the current font and scale=1
     * unless you pass a different scale.
     *
     * @param h     Display handle.
     * @param x,y   Top-left of the wrapping rectangle.
     * @param w,hgt Width and height of the wrapping rectangle (pixels).
     * @param text  NUL-terminated ASCII string.
     * @param on    true sets pixels, false clears them.
     * @param scale Integer scale factor.
     * @return ESP_OK on success.
     */
    esp_err_t ssd1306_draw_text_wrapped_scaled(ssd1306_handle_t h, int x, int y,
                                               int w, int hgt, const char *text,
                                               bool on, int scale);

    /**
     * @brief Send the current framebuffer to the display (flush).
     *
     * @param h Display handle.
     * @return ESP_OK on success.
     */
    esp_err_t ssd1306_display(ssd1306_handle_t h);
/**
     * @brief 画bitmap图
     * @param h     Display handle.
     * @param x,y   起点的xy坐标
     * @param bitmap 位图文件
     * @param width height  位图文件的宽度和高度
     * @return ESP_OK on success.
     */
    esp_err_t ssd1306_draw_bitmap(ssd1306_handle_t h, int x, int y, const uint8_t *bitmap, int width, int height);

#ifdef __cplusplus
}
#endif
