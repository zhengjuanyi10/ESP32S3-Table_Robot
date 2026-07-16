/*
 * 功能：ILI9488 独立示例，依次显示五色测试和蓝白竖屏简易首页。
 * 配置：复用工程 menuconfig 中的显示引脚、GPIO 时序和 PWM 背光参数。
 * 约束：只调用 display_ili9488 基础接口，不依赖 display_app 的 UI 函数。
 */

#include <ctype.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "display/display_ili9488.h"

#define EXAMPLE_BLUE ILI9488_RGB565(45, 120, 195)
#define EXAMPLE_LINE_THICKNESS 3

static const char *TAG = "display_example";

static bool parse_brightness(const char *line, uint8_t *brightness)
{
    char *end;
    const long value = strtol(line, &end, 10);
    if (end == line) {
        return false;
    }
    while (*end != '\0' && isspace((unsigned char)*end)) {
        ++end;
    }
    if (*end != '\0' || value < 0 || value > 100) {
        return false;
    }
    *brightness = (uint8_t)value;
    return true;
}

static esp_err_t draw_thick_frame(int x, int y, int width, int height,
                                  int thickness, uint16_t color)
{
    esp_err_t err = ili9488_display_fill_rect(
        x, y, width, thickness, color);
    if (err == ESP_OK) {
        err = ili9488_display_fill_rect(
            x, y + height - thickness, width, thickness, color);
    }
    if (err == ESP_OK) {
        err = ili9488_display_fill_rect(
            x, y + thickness, thickness, height - 2 * thickness, color);
    }
    if (err == ESP_OK) {
        err = ili9488_display_fill_rect(
            x + width - thickness, y + thickness, thickness,
            height - 2 * thickness, color);
    }
    return err;
}

static esp_err_t draw_simple_home(void)
{
    static const char *labels[] = {
        "COLOR", "TEXT", "SHAPE", "IMAGE",
    };
    const int content_x = 8;
    const int content_y = 64;
    const int content_width = 304;
    const int content_height = 408;
    const int split_x = content_x + content_width / 2 -
                        EXAMPLE_LINE_THICKNESS / 2;
    const int split_y = content_y + content_height / 2 -
                        EXAMPLE_LINE_THICKNESS / 2;

    /* 首页使用 320x480 竖屏。 */
    esp_err_t err = ili9488_display_set_rotation(
        ILI9488_DISPLAY_ROTATION_0);
    if (err == ESP_OK) {
        err = ili9488_display_fill(ILI9488_COLOR_WHITE);
    }

    /* 顶部蓝色标题栏。 */
    if (err == ESP_OK) {
        err = ili9488_display_fill_rect(8, 8, 304, 48, EXAMPLE_BLUE);
    }
    if (err == ESP_OK) {
        err = ili9488_display_draw_text(
            89, 25, "DISPLAY DEMO", 2, ILI9488_COLOR_WHITE,
            EXAMPLE_BLUE, true);
    }

    /* 下方只画一个粗外框和共享十字线。 */
    if (err == ESP_OK) {
        err = draw_thick_frame(
            content_x, content_y, content_width, content_height,
            EXAMPLE_LINE_THICKNESS, EXAMPLE_BLUE);
    }
    if (err == ESP_OK) {
        err = ili9488_display_fill_rect(
            content_x, split_y, content_width, EXAMPLE_LINE_THICKNESS,
            EXAMPLE_BLUE);
    }
    if (err == ESP_OK) {
        err = ili9488_display_fill_rect(
            split_x, content_y, EXAMPLE_LINE_THICKNESS, content_height,
            EXAMPLE_BLUE);
    }

    /* 四个标签直接使用驱动内置 5x7 字体。 */
    for (int index = 0; index < 4 && err == ESP_OK; ++index) {
        const int column = index % 2;
        const int row = index / 2;
        const int center_x = content_x +
                             content_width * (2 * column + 1) / 4;
        const int center_y = content_y +
                             content_height * (2 * row + 1) / 4;
        const int label_width =
            (int)(strlen(labels[index]) * 12U - 2U);
        err = ili9488_display_draw_text(
            center_x - label_width / 2, center_y - 7,
            labels[index], 2, ILI9488_COLOR_BLACK,
            ILI9488_COLOR_WHITE, true);
    }
    return err;
}

void app_main(void)
{
    ESP_ERROR_CHECK(ili9488_display_init());
    ESP_ERROR_CHECK(ili9488_display_set_brightness(40));

    ESP_LOGI(TAG, "Drawing five-color test pattern");
    ESP_ERROR_CHECK(ili9488_display_draw_test_pattern());
    vTaskDelay(pdMS_TO_TICKS(1000));

    ESP_LOGI(TAG, "Drawing standalone portrait home page");
    ESP_ERROR_CHECK(draw_simple_home());

    /* 串口输入 0~100 并回车，将背光设置为对应百分比。 */
    char line[32];
    printf("Brightness %% > ");
    fflush(stdout);
    while (true) {
        if (fgets(line, sizeof(line), stdin) == NULL) {
            clearerr(stdin);
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        uint8_t brightness;
        if (!parse_brightness(line, &brightness)) {
            ESP_LOGW(TAG, "Enter an integer from 0 to 100");
        } else {
            esp_err_t err = ili9488_display_set_brightness(brightness);
            if (err == ESP_OK) {
                ESP_LOGI(TAG, "Backlight brightness set to %u%%", brightness);
            } else {
                ESP_LOGE(TAG, "Backlight update failed: %s",
                         esp_err_to_name(err));
            }
        }
        printf("Brightness %% > ");
        fflush(stdout);
    }
}
