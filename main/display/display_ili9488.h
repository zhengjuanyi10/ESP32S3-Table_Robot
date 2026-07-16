/*
 * 功能：声明 ILI9488 初始化、面板状态、背光和基础绘图接口。
 * 配置：GPIO 与时序由 Table Robot/ILI9488 display menuconfig 提供。
 * 约定：接口执行同步基础绘图，不维护页面布局和页面状态。
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    ILI9488_DISPLAY_ROTATION_0,
    ILI9488_DISPLAY_ROTATION_90,
    ILI9488_DISPLAY_ROTATION_180,
    ILI9488_DISPLAY_ROTATION_270,
} ili9488_display_rotation_t;

/* RGB565 color helper and common UI colors. */
#define ILI9488_RGB565(red, green, blue) \
    ((uint16_t)((((uint16_t)(red) & 0xF8U) << 8) | \
                (((uint16_t)(green) & 0xFCU) << 3) | \
                ((uint16_t)(blue) >> 3)))

#define ILI9488_COLOR_BLACK   ((uint16_t)0x0000)
#define ILI9488_COLOR_NAVY    ((uint16_t)0x000F)
#define ILI9488_COLOR_BLUE    ((uint16_t)0x001F)
#define ILI9488_COLOR_GREEN   ((uint16_t)0x07E0)
#define ILI9488_COLOR_CYAN    ((uint16_t)0x07FF)
#define ILI9488_COLOR_RED     ((uint16_t)0xF800)
#define ILI9488_COLOR_MAGENTA ((uint16_t)0xF81F)
#define ILI9488_COLOR_YELLOW  ((uint16_t)0xFFE0)
#define ILI9488_COLOR_WHITE   ((uint16_t)0xFFFF)
#define ILI9488_COLOR_GRAY    ((uint16_t)0x8410)

/*
 * Initializes the known-good GPIO 4-wire transport, panel and backlight.
 * Touch GPIOs are not configured or driven here.
 * The default orientation is landscape (480 x 320). This function is
 * idempotent and does not create any background task.
 */
esp_err_t ili9488_display_init(void);
esp_err_t ili9488_display_deinit(void);
bool ili9488_display_is_initialized(void);

esp_err_t ili9488_display_set_backlight(bool enabled);
esp_err_t ili9488_display_set_brightness(uint8_t percent);

esp_err_t ili9488_display_set_rotation(ili9488_display_rotation_t rotation);
uint16_t ili9488_display_width(void);
uint16_t ili9488_display_height(void);

/*
 * Synchronous drawing interfaces. Coordinates are clipped to the screen.
 * RGB data uses CPU-native RGB565, and the input buffer may be released
 * immediately after the call returns.
 */
esp_err_t ili9488_display_draw_pixel(int x, int y, uint16_t color);
esp_err_t ili9488_display_draw_hline(int x, int y, int width,
                                     uint16_t color);
esp_err_t ili9488_display_draw_vline(int x, int y, int height,
                                     uint16_t color);
esp_err_t ili9488_display_draw_line(int x0, int y0, int x1, int y1,
                                    uint16_t color);
esp_err_t ili9488_display_draw_rect(int x, int y, int width, int height,
                                    uint16_t color);
esp_err_t ili9488_display_draw_circle(int center_x, int center_y, int radius,
                                      uint16_t color);
esp_err_t ili9488_display_fill_circle(int center_x, int center_y, int radius,
                                      uint16_t color);
esp_err_t ili9488_display_draw_bitmap(int x, int y, int width, int height,
                                     const uint16_t *rgb565_pixels);

/*
 * Direct RGB888 input path. The driver converts each pixel to the LCD bus
 * format internally, so the input buffer may be released after the call.
 */
esp_err_t ili9488_display_draw_rgb888(int x, int y, int width, int height,
                                     const uint8_t *rgb888_pixels);
esp_err_t ili9488_display_fill_rect(int x, int y, int width, int height,
                                    uint16_t rgb565_color);
esp_err_t ili9488_display_fill(uint16_t rgb565_color);

/* Draws a 1-bit MSB-first glyph/icon; zero stride means tightly packed. */
esp_err_t ili9488_display_draw_mono_bitmap(
    int x, int y, int width, int height, const uint8_t *bitmap,
    size_t row_stride_bytes, uint16_t foreground, uint16_t background,
    bool transparent_background);

/* Built-in 5x7 uppercase ASCII font; scale must be at least 1. */
esp_err_t ili9488_display_draw_text(int x, int y, const char *text,
                                    unsigned scale, uint16_t foreground,
                                    uint16_t background,
                                    bool transparent_background);

/* Draws five permanent vertical RGB666 diagnostic bars. */
esp_err_t ili9488_display_draw_test_pattern(void);

#ifdef __cplusplus
}
#endif
