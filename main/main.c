#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "servo/robot_motion.h"

static const char *TAG = "table_robot";

void app_main(void)
{
    robot_motion_init();
    robot_motion_start_debug_console();

    ESP_LOGI(TAG, "Servo-only firmware ready");
    ESP_LOGI(TAG, "Actions: 1=right wave, 2=left wave, 3=raise both hands");
    ESP_LOGI(TAG, "Diagnostics: diagnose, test, center, or <id> <degrees>");

    while (true) {
        robot_motion_poll();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
