#ifndef WS2812_DRIVER_H
#define WS2812_DRIVER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// 初始化WS2812
void ws2812_init(int gpio_num);

// 设置单个LED颜色
void ws2812_set_color(uint8_t r, uint8_t g, uint8_t b);

// 清空LED
void ws2812_clear(void);

// 彩虹呼吸灯效果
void ws2812_rainbow_breathing(uint16_t period_ms);

// 单色呼吸灯效果
void ws2812_color_breathing(uint8_t r, uint8_t g, uint8_t b, uint16_t period_ms);

// 单色常亮
void ws2812_solid_color(uint8_t r, uint8_t g, uint8_t b);

#ifdef __cplusplus
}
#endif

#endif // WS2812_DRIVER_H