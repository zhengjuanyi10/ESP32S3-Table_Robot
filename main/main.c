/*
 * 功能：启动 display_app，通过串口输入 0~100 调节背光百分比，
 *       同时以独立任务轮询电容触摸屏。
 * 配置：使用 ESP-IDF 控制台 UART0，当前 sdkconfig 波特率为 115200。
 * 输入：发送一个十进制整数并回车，例如 35 表示背光亮度 35%。
 */

#include <ctype.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "display/display_app.h"
#include "display/display_ili9488.h"

#if CONFIG_TABLE_ROBOT_TOUCH_ENABLE
#include "control/touch/touch_chip.h"
#endif

static const char *TAG = "table_robot";

static bool parse_brightness(const char *line, uint8_t *brightness)
{
    char *end;
    const long value = strtol(line, &end, 10);
    if (end == line) {
        return false;
    }

    /* 回车、换行和空格允许出现在数字后，其他字符视为无效输入。 */
    while (*end != '\0' && isspace((unsigned char)*end)) {
        ++end;
    }
    if (*end != '\0' || value < 0 || value > 100) {
        return false;
    }

    *brightness = (uint8_t)value;
    return true;
}

#if CONFIG_TABLE_ROBOT_TOUCH_ENABLE
static void touch_poll_task(void *pvParameters)
{
    touch_sample_t sample;
    bool was_pressed = false;
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period =
        pdMS_TO_TICKS(CONFIG_TABLE_ROBOT_TOUCH_POLL_PERIOD_MS);

    while (true) {
        vTaskDelayUntil(&last_wake, period);
        esp_err_t err = touch_chip_read_sample(&sample);
        if (err != ESP_OK || !sample.online) {
            was_pressed = false;
            continue;
        }

        if (sample.pressed) {
            if (!was_pressed) {
                /* 新按下——只触发一次导航，避免按住时反复跳转。 */
                was_pressed = true;
                ESP_LOGI(TAG, "Touch at (%u, %u)  ctrl=%d",
                         sample.raw_x, sample.raw_y, (int)sample.controller);
                display_app_handle_touch((int)sample.raw_x, (int)sample.raw_y);
            }
        } else {
            was_pressed = false;
        }
    }
}
#endif

void app_main(void)
{
    esp_err_t err = display_app_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "display_app initialization failed: %s",
                 esp_err_to_name(err));
        return;
    }

#if CONFIG_TABLE_ROBOT_TOUCH_ENABLE
    touch_chip_boot_probe();
    xTaskCreate(touch_poll_task, "touch_poll", 4096, NULL, 5, NULL);
#endif

    ESP_LOGI(TAG, "display_app running");
    ESP_LOGI(TAG, "UART brightness control ready (0-100)");

    char line[32];
    printf("Brightness %% > ");
    fflush(stdout);
    while (true) {
        if (fgets(line, sizeof(line), stdin) == NULL) {
            /* 控制台暂时无数据时让出 CPU，并清除 EOF 状态后继续等待。 */
            clearerr(stdin);
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        uint8_t brightness;
        if (!parse_brightness(line, &brightness)) {
            ESP_LOGW(TAG, "Invalid brightness; enter an integer from 0 to 100");
        } else {
            err = ili9488_display_set_brightness(brightness);
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
