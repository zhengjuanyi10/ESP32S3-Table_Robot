#include <stdbool.h>

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "power_button.h"

/* 黑帽按键模块：S 接 GPIO2，按下时输出高电平。 */
#define POWER_BUTTON_GPIO GPIO_NUM_2
#define DEBOUNCE_MS 30

static bool s_raw_pressed;
static bool s_stable_pressed;
static TickType_t s_last_change;

void power_button_init(void)
{
    const gpio_config_t config = {
        .pin_bit_mask = 1ULL << POWER_BUTTON_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
    };
    gpio_config(&config);

    /* 记录当前电平，避免上电时误判一次按键。 */
    s_raw_pressed = gpio_get_level(POWER_BUTTON_GPIO) == 1;
    s_stable_pressed = s_raw_pressed;
    s_last_change = xTaskGetTickCount();
}

power_button_event_t power_button_poll(void)
{
    const TickType_t now = xTaskGetTickCount();
    const bool pressed = gpio_get_level(POWER_BUTTON_GPIO) == 1;

    /* 先记录原始电平变化，再等待 30 ms 后确认，过滤机械抖动。 */
    if (pressed != s_raw_pressed) {
        s_raw_pressed = pressed;
        s_last_change = now;
    }
    if (s_raw_pressed != s_stable_pressed &&
        now - s_last_change >= pdMS_TO_TICKS(DEBOUNCE_MS)) {
        s_stable_pressed = s_raw_pressed;
        if (!s_stable_pressed) {
            return POWER_BUTTON_SHORT_PRESS;
        }
    }
    return POWER_BUTTON_NONE;
}
