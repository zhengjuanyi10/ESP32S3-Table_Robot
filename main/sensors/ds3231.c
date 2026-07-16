#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "ds3231.h"
#include "i2c_bus.h"

/* DS3231 使用标准 7 位 I2C 地址 0x68。 */
#define DS3231_ADDRESS 0x68

/* RTC 在共享总线上的设备句柄。 */
static i2c_master_dev_handle_t s_ds3231;

static int bcd_to_decimal(uint8_t value)
{
    /* DS3231 寄存器用 BCD 编码，例如 0x25 表示十进制 25。 */
    return ((value >> 4) * 10) + (value & 0x0F);
}

static uint8_t decimal_to_bcd(int value)
{
    return (uint8_t)(((value / 10) << 4) | (value % 10));
}

esp_err_t ds3231_init(void)
{
    /* 初始化可重复调用，不会在总线上重复添加同一设备。 */
    if (s_ds3231 != NULL) {
        return ESP_OK;
    }

    const i2c_device_config_t config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = DS3231_ADDRESS,
        .scl_speed_hz = 100000,
    };
    esp_err_t err = i2c_bus_init();
    return err == ESP_OK ? i2c_master_bus_add_device(i2c_bus_get_handle(), &config, &s_ds3231) : err;
}

esp_err_t ds3231_get_time(ds3231_time_t *time)
{
    /* 从 0x00 连续读取秒、分、时、星期、日、月、年七个寄存器。 */
    if (time == NULL || s_ds3231 == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    const uint8_t first_register = 0x00;
    uint8_t data[7] = {0};
    esp_err_t err = i2c_master_transmit_receive(s_ds3231, &first_register, 1, data, sizeof(data), 100);
    if (err != ESP_OK) {
        return err;
    }

    /* 同时兼容 DS3231 的 24 小时制和 12 小时制寄存器格式。 */
    int hour = bcd_to_decimal(data[2] & ((data[2] & 0x40) ? 0x1F : 0x3F));
    if (data[2] & 0x40) {
        hour = (hour == 12) ? 0 : hour;
        hour += (data[2] & 0x20) ? 12 : 0;
    }

    time->second = bcd_to_decimal(data[0] & 0x7F);
    time->minute = bcd_to_decimal(data[1] & 0x7F);
    time->hour = hour;
    time->day = bcd_to_decimal(data[4] & 0x3F);
    time->month = bcd_to_decimal(data[5] & 0x1F);
    time->year = 2000 + bcd_to_decimal(data[6]);
    return ESP_OK;
}

esp_err_t ds3231_set_time(const ds3231_time_t *time)
{
    if (time == NULL || s_ds3231 == NULL ||
        time->year < 2000 || time->year > 2099 ||
        time->month < 1 || time->month > 12 ||
        time->day < 1 || time->day > 31 ||
        time->hour < 0 || time->hour > 23 ||
        time->minute < 0 || time->minute > 59 ||
        time->second < 0 || time->second > 59) {
        return ESP_ERR_INVALID_ARG;
    }

    /* 从 0x00 连续写入秒、分、时、星期、日、月、年七个寄存器。 */
    const uint8_t data[] = {
        0x00,
        decimal_to_bcd(time->second),
        decimal_to_bcd(time->minute),
        decimal_to_bcd(time->hour),
        1, /* 星期字段当前未使用，固定写星期一。 */
        decimal_to_bcd(time->day),
        decimal_to_bcd(time->month),
        decimal_to_bcd(time->year - 2000),
    };
    return i2c_master_transmit(s_ds3231, data, sizeof(data), 100);
}

esp_err_t ds3231_set_to_build_time(void)
{
    static const char *const months[] = {
        "Jan", "Feb", "Mar", "Apr", "May", "Jun",
        "Jul", "Aug", "Sep", "Oct", "Nov", "Dec",
    };
    char month_name[4] = {0};
    ds3231_time_t time = {0};

    /* __DATE__/__TIME__ 由编译器提供，例如 "Jul 14 2026"、"19:30:00"。 */
    if (sscanf(__DATE__, "%3s %d %d", month_name, &time.day, &time.year) != 3 ||
        sscanf(__TIME__, "%d:%d:%d", &time.hour, &time.minute, &time.second) != 3) {
        return ESP_FAIL;
    }
    for (int index = 0; index < 12; ++index) {
        if (strcmp(month_name, months[index]) == 0) {
            time.month = index + 1;
            return ds3231_set_time(&time);
        }
    }
    return ESP_FAIL;
}
