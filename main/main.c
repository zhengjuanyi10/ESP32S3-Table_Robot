/*
 * 功能：启动 display_app，并通过串口输入 0~100 调节背光百分比。
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

void app_main(void)
{
    esp_err_t err = display_app_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "display_app initialization failed: %s",
                 esp_err_to_name(err));
        return;
    }

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
