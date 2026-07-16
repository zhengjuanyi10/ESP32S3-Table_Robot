#pragma once

#include "esp_err.h"

/* SHT30 单次测量结果：摄氏度和相对湿度百分比。 */
typedef struct {
    float temperature_c;
    float humidity_percent;
} sht30_measurement_t;

esp_err_t sht30_init(void); /* 注册默认地址 0x44 的 SHT30。 */
esp_err_t sht30_get_measurement(sht30_measurement_t *measurement);
