#pragma once

/* 按键事件：短按在松开时产生。 */
typedef enum {
    POWER_BUTTON_NONE,
    POWER_BUTTON_SHORT_PRESS,
} power_button_event_t;

/* 初始化按键：GPIO2，模块按下为高电平。 */
void power_button_init(void);

/* 在主循环中周期调用，返回一次按键事件。 */
power_button_event_t power_button_poll(void);
