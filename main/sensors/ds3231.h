/*
 * 功能：声明 DS3231 初始化、读取时间、设置时间和编译时间校时接口。
 * 配置：默认地址 0x68，年份范围 2000~2099，日期时间使用 24 小时制。
 */

#pragma once

#include "esp_err.h"

/* DS3231 读取到的公历时间，使用 24 小时制。 */
typedef struct {
    int year;
    int month;
    int day;
    int hour;
    int minute;
    int second;
} ds3231_time_t;

/* 注册 RTC、读取/写入日期时间，以及使用编译时间进行一次性校时。 */
esp_err_t ds3231_init(void);
esp_err_t ds3231_get_time(ds3231_time_t *time);
esp_err_t ds3231_set_time(const ds3231_time_t *time);
esp_err_t ds3231_set_to_build_time(void);
