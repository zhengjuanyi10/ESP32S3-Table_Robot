/* DS3231 实时时钟驱动接口。 */

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

esp_err_t ds3231_init(void);                 /* 注册地址 0x68 的 RTC 设备。 */
esp_err_t ds3231_get_time(ds3231_time_t *time); /* 读取一次当前日期时间。 */
esp_err_t ds3231_set_time(const ds3231_time_t *time); /* 写入日期时间并使用 24 小时制。 */
esp_err_t ds3231_set_to_build_time(void); /* 用本次固件编译时间校时。 */
